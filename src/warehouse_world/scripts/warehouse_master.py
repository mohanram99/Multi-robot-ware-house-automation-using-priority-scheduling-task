#!/usr/bin/env python3
"""
WAREHOUSE MASTER — Priority Scheduler + WebSocket backend for the UI.

"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

import asyncio, json, math, threading, time, heapq, itertools, sys, os
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drone_position_controller import DronePositionController

try:
    import websockets
except ImportError:
    print("ERROR: pip install websockets --break-system-packages")
    sys.exit(1)

# ═══════════════════════════ CONSTANTS ═══════════════════════
WORLD_NAME   = "multi_robot_warehouse"

DRONE_PAD    = (0.0, -10.0, 0.75)
PAD_WEST_RELEASE_X = -2.5
CRUISE_ALT   = 3.0
RETURN_ALT   = 1.3
FRAME_TOP_Z  = 2.0
CLEARANCE    = 0.08
NORMAL_H     = 0.26
CARRY_OFFSET = 0.26
DRONE_DROP   = (10.0, -10.0, 0.5)
DRONE_BOX_DROP_Z = 0.16

AMR_HOME      = (-11.0, -10.0)
AMR2_HOME     = (-8.5, -10.0)
BIG_HOME      = (-5.5, -10.0)

SOUTH_Y       = -7.8

SOUTH_LANE_OUT    = -7.4
SOUTH_LANE_RETURN = -8.15
EXIT_LANE = {'amr': -9.0, 'amr2': -8.3, 'big_amr': -7.6}
MID_Y         = -3.0
NORTH_CORR_Y  = 3.0
NORTH_LANE_Y  = 9.0
V_AISLES      = [-6.0, 0.0, 6.0]

STAGE_X       = 6.0
HOLD_Y        = -5.0
STAGE_SLOT_DY = 1.5
AMR_CARRY_Z   = 0.35
BIG_CARRY_Z   = 0.47

AMR_DROP      = (9.2, -9.7)
AMR2_DROP     = (10.0, -9.7)
BIG_DROP      = (10.8, -9.7)
AMR_BOX_DROP_Z = 0.13
BIG_BOX_DROP_Z = 0.17

GAP_X_RANGE   = (2.5, 7.5)
GAP_Y         = SOUTH_Y - 0.8

PRIO = {'urgent': 0, 'normal': 1, 'low': 2}

SPEED_MULT = 1.0

def hz(bz, bh): return round(bz + bh/2 + CLEARANCE, 3)

# ═══════════════════════════ BOX DATABASE ════════════════════
DRONE_BOXES, AMR_BOXES, BIG_BOXES = {}, {}, {}

for col, sx in [('1',-9),('2',-3),('3',3),('4',9)]:
    for s, xo in [('la',-0.45),('lb',-0.45),('ra',0.45),('rb',0.45)]:
        pass

for row, sy in [('2',0.0),('3',-6.0)]:
    for col, sx in [('1',-9),('2',-3),('3',3),('4',9)]:
        for s, xo, yo in [('la',-0.45,-0.45),('lb',-0.45,0.45),
                           ('ra', 0.45,-0.45),('rb', 0.45,0.45)]:
            DRONE_BOXES[f'box_{row}_{col}_high_{s}'] = {
                'x': sx+xo, 'y': sy+yo, 'hover_z': hz(1.655, NORMAL_H)}
            south_facing = (row == '3' and s in ('la', 'ra'))
            if south_facing:
                ay = -7.0
                outer = 'south'
                ax = sx + xo
            else:
                ay = -1.8 if row == '2' else -4.2
                outer = None
                ax = sx
            AMR_BOXES[f'box_{row}_{col}_low_{s}'] = {
                'x': sx+xo, 'y': sy+yo, 'approach': (ax, ay),
                'outer': outer}

for col, sx in [('1',-9),('2',-3),('3',3),('4',9)]:
    for s, xo, yo in [('ll',-0.45,-0.45),('lr',-0.45,0.45),
                       ('rl', 0.45,-0.45),('rr', 0.45,0.45)]:
        north_facing = s in ('lr', 'rr')
        if north_facing:
            ay = NORTH_LANE_Y - 1.5
            outer = 'north'
            ax = sx + xo
        else:
            ay = 4.7
            outer = None
            ax = sx
        BIG_BOXES[f'bigbox_1_{col}_{s}'] = {
            'x': sx+xo, 'y': 6.0+yo, 'approach': (ax, ay),
            'outer': outer}

ALL_BOXES = {**DRONE_BOXES, **AMR_BOXES, **BIG_BOXES}


def robot_for(box):
    if box in DRONE_BOXES: return 'drone'
    if box in BIG_BOXES:   return 'big_amr'
    try:
        parts = box.split('_')
        if len(parts) < 3:
            raise ValueError(f"unexpected box name format: {box}")
        col = int(parts[2])
        assigned = 'amr' if (col % 2 == 1) else 'amr2'
        return assigned
    except Exception as e:
        log(f"[WARN] robot_for({box}) parse failed ({e}) -> falling back to amr")
        return 'amr'


def home_z_for(key):
    return {'amr': 0.02, 'amr2': 0.02, 'big_amr': 0.0}.get(key, 0.02)

MOVE_Z_OFFSET = 0.0

LIDAR_TOPICS = {
    'amr':     '/scan',
    'amr2':    '/scan2',
    'big_amr': '/big_amr/scan',
}
LIDAR_ALERT_DIST = 1.0
LIDAR_SELF_MIN   = 0.05

# ═══════════════════════════ SHARED STATE ════════════════════
STATE = {
    'drone':   {'x':0.0,'y':-10.0,'z':0.75,'status':'IDLE','job':None,'step':''},
    'amr':     {'x':-11.0,'y':-10.0,'status':'IDLE','job':None,'step':'','carrying':False,'vx':0.0,'vy':0.0,'lidar_min':None,'lidar_alert':False,'phase':None},
    'amr2':    {'x':-8.5,'y':-10.0,'status':'IDLE','job':None,'step':'','carrying':False,'vx':0.0,'vy':0.0,'lidar_min':None,'lidar_alert':False,'phase':None},
    'big_amr': {'x':-5.5,'y':-10.0,'status':'IDLE','job':None,'step':'','carrying':False,'vx':0.0,'vy':0.0,'lidar_min':None,'lidar_alert':False,'phase':None},
    'queue': [], 'log': [], 'done': [],
    'delivery_occupied': None,
    'delivery_waiters': {},
}
STATE_LOCK = threading.Lock()

def log(msg):
    with STATE_LOCK:
        STATE['log'].append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        if len(STATE['log']) > 60: STATE['log'].pop(0)
    print(msg)


# ═══════════════════════════ DRONE NODE ══════════════════════
class DroneCtl(DronePositionController):
    def __init__(self):
        super().__init__()
        self.enable_pub = self.create_publisher(Bool, '/drone/enable', 10)
        self._pose_cli = self.create_client(
            SetEntityPose, f'/world/{WORLD_NAME}/set_pose')
        self.busy = False
        self._carrying = None
        self._done_cb  = None
        threading.Thread(target=self._carry_loop, daemon=True).start()

    def _rst(self):
        self.POSITION_THRESHOLD = 0.35
        self.pid_x.limit = 1.0; self.pid_y.limit = 1.0; self.pid_z.limit = 1.5

    def _one_shot(self, d, cb):
        def f():
            t.cancel(); cb()
        t = self.create_timer(d, f)

    def _pose_req(self, name, x, y, z):
        r = SetEntityPose.Request()
        r.entity = Entity(); r.entity.name = name; r.entity.type = Entity.MODEL
        r.pose = Pose()
        r.pose.position.x = float(x); r.pose.position.y = float(y)
        r.pose.position.z = float(z); r.pose.orientation.w = 1.0
        return r

    def _teleport(self, name, x, y, z):
        res = subprocess.run(['gz','service','-s',
            f'/world/{WORLD_NAME}/set_pose','--reqtype','gz.msgs.Pose',
            '--reptype','gz.msgs.Boolean','--timeout','300','--req',
            f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'],
            capture_output=True, text=True)
        return res.returncode == 0

    def _carry_loop(self):
        self._pose_cli.wait_for_service()
        while True:
            b = self._carrying
            if b:
                self._pose_cli.call_async(self._pose_req(
                    b, self.current_x, self.current_y,
                    self.current_z - CARRY_OFFSET))
                time.sleep(0.03)
            else:
                time.sleep(0.05)

    def _upd(self, step):
        with STATE_LOCK:
            STATE['drone']['step'] = step
        log(f"[DRONE] {step}")

    def run_job(self, box, done_cb):
        self.busy = True; self._done_cb = done_cb
        self._b = DRONE_BOXES[box]; self._box_name = box
        with STATE_LOCK:
            STATE['drone']['status'] = 'BUSY'; STATE['drone']['job'] = box
        m = Bool(); m.data = True; self.enable_pub.publish(m)
        self._rst(); self._upd('Ascending')
        self.set_goal(*DRONE_PAD[:2], CRUISE_ALT, reached_cb=self._s2)

    def _s2(self):
        self._rst(); b = self._b; self._upd('Flying to box')
        self.set_goal(b['x'], b['y'], CRUISE_ALT, reached_cb=self._s3)

    def _s3(self):
        self._rst(); b = self._b; self._upd('Descending to frame top')
        self.set_goal(b['x'], b['y'], FRAME_TOP_Z, reached_cb=self._s4)

    def _s4(self):
        self._rst(); self.pid_z.limit = 0.3
        b = self._b; self._upd('Slow descent to box')
        self.set_goal(b['x'], b['y'], b['hover_z'], reached_cb=self._s5)

    def _s5(self):
        self._rst(); self._upd(f'ATTACH {self._box_name}')
        if self._teleport(self._box_name, self.current_x, self.current_y,
                          self.current_z - CARRY_OFFSET):
            self._carrying = self._box_name
        else:
            log('[DRONE] attach teleport FAILED'); self._finish(); return
        self._one_shot(1.0, self._s6)

    def _s6(self):
        self._rst(); b = self._b; self._upd('Ascending with box')
        self.set_goal(b['x'], b['y'], CRUISE_ALT, reached_cb=self._s7)

    def _s7(self):
        self._rst(); self._upd('To delivery')
        self.set_goal(DRONE_DROP[0], DRONE_DROP[1], CRUISE_ALT,
                      reached_cb=self._s8)

    def _s8(self):
        self._rst(); self._upd('Descending to delivery')
        self.set_goal(*DRONE_DROP, reached_cb=self._s9)

    def _s9(self):
        self._upd('DETACH — placing box')
        self._carrying = None; time.sleep(0.1)
        self._teleport(self._box_name, DRONE_DROP[0], DRONE_DROP[1],
                       DRONE_BOX_DROP_Z)
        self._s10()

    def _s10(self):
        self._rst(); self._upd('Return ascend')
        self.set_goal(DRONE_DROP[0], DRONE_DROP[1], RETURN_ALT,
                      reached_cb=self._s11)

    def _s11(self):
        self._rst(); self._upd('Flying to pad')
        self.set_goal(DRONE_PAD[0], DRONE_PAD[1], RETURN_ALT,
                      reached_cb=self._s12)

    def _s12(self):
        self._rst(); self.pid_z.limit = 0.3
        self.POSITION_THRESHOLD = 0.20; self._upd('Landing')
        self.set_goal(*DRONE_PAD, reached_cb=self._s13)

    def _s13(self):
        self.stop()
        m = Bool(); m.data = False; self.enable_pub.publish(m)
        self._upd('Parked — motors off')
        self._finish()

    def _finish(self):
        self.busy = False
        with STATE_LOCK:
            STATE['done'].append(self._box_name)
            STATE['drone']['status'] = 'IDLE'; STATE['drone']['job'] = None
        cb = self._done_cb; self._done_cb = None
        if cb: cb('drone')


# ═══════════════════════════ AMR NODE (teleport L-path) ══════
class AmrCtl(Node):
    BASE_STEP = 0.03

    @property
    def STEP(self):
        return self.BASE_STEP * SPEED_MULT  #Speed from ui

    def __init__(self, key, cmd_topic, odom_topic, home, carry_z,
                 drop_xy, drop_z, spawn_yaw=1.5708, model_name=None):
        super().__init__(f'{key}_ctl')
        self.key = key
        self.model_name = model_name or key
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._pose_cli = self.create_client(
            SetEntityPose, f'/world/{WORLD_NAME}/set_pose')
        self.home, self.carry_z = home, carry_z
        self.drop_xy, self.drop_z = drop_xy, drop_z
        self.z_robot = home_z_for(key)
        self.x, self.y = home[0], home[1]
        self.yaw = spawn_yaw
        self.busy = False
        self._carrying = None
        self._wps = []
        self._phase = None
        self._box_name = None
        self._done_cb = None
        self._outer = None
        self._hist = []
        self._yield_wait = 0
        self._released_mutex = False
        self._stuck = 0
        self._forcing = False

        self._lidar_min = None
        self._lidar_last_log = 0.0
        scan_topic = LIDAR_TOPICS.get(key)
        if scan_topic:
            self.create_subscription(
                LaserScan, scan_topic, self._scan_cb, 10)

        self.create_timer(0.05, self._tick)

    def _scan_cb(self, msg: LaserScan):
        best_d = float('inf'); best_ang = 0.0
        ang = msg.angle_min
        rmin = max(msg.range_min, LIDAR_SELF_MIN)
        rmax = msg.range_max
        for r in msg.ranges:
            if r == r and rmin <= r <= rmax:
                if r < best_d:
                    best_d = r; best_ang = ang
            ang += msg.angle_increment

        if best_d == float('inf'):
            self._lidar_min = None
            with STATE_LOCK:
                STATE[self.key]['lidar_min'] = None
                STATE[self.key]['lidar_alert'] = False
            return

        self._lidar_min = (best_d, best_ang)
        alert = best_d < LIDAR_ALERT_DIST
        with STATE_LOCK:
            STATE[self.key]['lidar_min'] = round(best_d, 2)
            STATE[self.key]['lidar_alert'] = alert

        if alert:
            now = time.time()
            if now - self._lidar_last_log > 1.0:
                self._lidar_last_log = now
                deg = math.degrees(best_ang)
                log(f"[{self.key.upper()}] LiDAR: obstacle {best_d:.2f}m "
                    f"@ {deg:+.0f}\u00b0")

    def _pose_req(self, name, x, y, z, yaw=0.0):
        r = SetEntityPose.Request()
        r.entity = Entity(); r.entity.name = name; r.entity.type = Entity.MODEL
        r.pose = Pose()
        r.pose.position.x = float(x); r.pose.position.y = float(y)
        r.pose.position.z = float(z)
        r.pose.orientation.z = math.sin(yaw/2.0)
        r.pose.orientation.w = math.cos(yaw/2.0)
        return r

    def _place_robot(self, moving=False):
        z = self.z_robot + (MOVE_Z_OFFSET if moving else 0.0)
        self._pose_cli.call_async(
            self._pose_req(self.model_name, self.x, self.y,
                           z, self.yaw))
        if self._carrying:
            self._pose_cli.call_async(
                self._pose_req(self._carrying, self.x, self.y,
                               self.carry_z, self.yaw))
        with STATE_LOCK:
            old_x = STATE[self.key]['x']; old_y = STATE[self.key]['y']
            STATE[self.key]['vx'] = round(self.x - old_x, 4)
            STATE[self.key]['vy'] = round(self.y - old_y, 4)
            STATE[self.key]['x'] = round(self.x, 2)
            STATE[self.key]['y'] = round(self.y, 2)
            STATE[self.key]['carrying'] = self._carrying is not None
            STATE[self.key]['phase'] = self._phase
        if not self._hist or math.hypot(self.x - self._hist[-1][0],
                                        self.y - self._hist[-1][1]) > 0.02:
            self._hist.append((self.x, self.y))
            if len(self._hist) > 40:
                self._hist.pop(0)

    @staticmethod
    def _peer_delivery_bound(key):
        s = STATE.get(key, {})
        return s.get('carrying', False) and s.get('phase') in ('stage', 'deliver')

    @staticmethod
    def _peer_carrying(key):
        return STATE.get(key, {}).get('carrying', False)

    def _upd(self, step):
        with STATE_LOCK: STATE[self.key]['step'] = step
        log(f"[{self.key.upper()}] {step}")

    def _route(self, frm, to):
        fx, fy = frm; tx, ty = to
        start_aisle = min([-6.0, 0.0, 6.0], key=lambda a: abs(a - fx))
        tgt_aisle = min([-6.0, 0.0, 6.0], key=lambda a: abs(a - tx))
        start_corr = self._corridor_for(fy)
        tgt_corr   = self._corridor_for(ty)
        wps = []
        if abs(fy - start_corr) > 0.3:
            wps.append((fx, start_corr))
        wps.append((start_aisle, start_corr))
        wps.append((start_aisle, tgt_corr))
        if tgt_aisle != start_aisle:
            wps.append((tgt_aisle, tgt_corr))
        wps.append((tgt_aisle, ty))
        wps.append((tx, ty))
        return wps

    @staticmethod
    def _corridor_for(y):
        corridors = [NORTH_CORR_Y, MID_Y, SOUTH_Y]
        return min(corridors, key=lambda c: abs(c - y))

    def run_job(self, box, done_cb):
        self.busy = True; self._done_cb = done_cb
        self._box_name = box
        info = ALL_BOXES[box]
        self._target = info['approach']
        self._outer = info.get('outer')
        with STATE_LOCK:
            STATE[self.key]['status'] = 'BUSY'; STATE[self.key]['job'] = box
        if self._outer:
            self._wps = self._route_perimeter((self.x, self.y),
                                               self._target, self._outer)
        else:
            self._wps = self._route((self.x, self.y), self._target)
        self._phase = 'out'
        self._upd(f'Navigating to {box}')

    def _route_perimeter(self, frm, to, side):
        fx, fy = frm; tx, ty = to
        wps = []
        if side == 'south':
            if abs(fy - SOUTH_Y) > 0.3:
                wps.append((fx, SOUTH_Y))
            wps.append((tx, SOUTH_Y))
            wps.append((tx, ty))
        else:
            start_aisle = min(V_AISLES, key=lambda a: abs(a - fx))
            start_corr = self._corridor_for(fy)
            if abs(fy - start_corr) > 0.3:
                wps.append((fx, start_corr))
            if abs(fx - start_aisle) > 0.3:
                wps.append((start_aisle, start_corr))
            wps.append((start_aisle, NORTH_LANE_Y))
            wps.append((tx, NORTH_LANE_Y))
            wps.append((tx, ty))
        return wps

    HORIZON   = 6
    SAFE_DIST = 1.0
    DELIV_YIELD_DIST = 2.5
    DELIV_CLEAR_DIST = 3.0
    OVERLAP_DIST     = 0.9
    STUCK_LIMIT      = 20
    OVERLAP_STUCK_LIMIT = 20
    GIVEWAY_CLEAR    = 1.1

    def _predict_peer(self, other, n):
        with STATE_LOCK:
            ox = STATE[other]['x']; oy = STATE[other]['y']
            ovx = STATE[other].get('vx', 0.0); ovy = STATE[other].get('vy', 0.0)
        return ox + ovx * n, oy + ovy * n

    def _rollout_min_dist(self, path_fn, n_steps, peers):
        best = float('inf')
        for n in range(1, n_steps + 1):
            mx, my = path_fn(n)
            for other in peers:
                px, py = self._predict_peer(other, n)
                d = math.hypot(mx - px, my - py)
                if d < best:
                    best = d
        return best

    def _proceed_path(self, tx, ty, dist, dx, dy):
        if dist < 1e-6:
            return lambda n: (self.x, self.y)
        ux, uy = dx / dist, dy / dist
        def f(n):
            d = min(self.STEP * n, dist)
            return (self.x + ux * d, self.y + uy * d)
        return f

    def _yield_path(self):
        gx_lo, gx_hi = GAP_X_RANGE
        gx = self.x if gx_lo <= self.x <= gx_hi else (gx_lo + gx_hi) / 2.0
        gy = GAP_Y
        sx, sy = gx - self.x, gy - self.y
        sdist = math.hypot(sx, sy)
        if sdist < 1e-6:
            return lambda n: (self.x, self.y)
        ux, uy = sx / sdist, sy / sdist
        def f(n):
            d = min(self.STEP * n, sdist)
            return (self.x + ux * d, self.y + uy * d)
        return f

    def _hold_path(self):
        return lambda n: (self.x, self.y)

    def _try_giveway(self, peers):
        def clear_of_peers(px, py, radius):
            with STATE_LOCK:
                for other in peers:
                    ox = STATE[other]['x']; oy = STATE[other]['y']
                    if math.hypot(ox - px, oy - py) < radius:
                        return False
            return True

        if len(self._hist) > 2:
            bx, by = self._hist[-3]
            if clear_of_peers(bx, by, self.GIVEWAY_CLEAR):
                ddx, ddy = bx - self.x, by - self.y
                d = math.hypot(ddx, ddy)
                if d > 1e-6:
                    step = min(self.STEP, d)
                    self.x += step * ddx / d
                    self.y += step * ddy / d
                    self.yaw = math.atan2(self.y - by, self.x - bx)
                    self._place_robot(moving=True)
                    self._hist.pop()
                return True
        return False

    def _tick(self):
        self._forcing = False

        if (self._phase == 'home' and not self._released_mutex
                and self.x < PAD_WEST_RELEASE_X):
            with STATE_LOCK:
                if STATE.get('delivery_occupied') == self.key:
                    STATE['delivery_occupied'] = None
            self._released_mutex = True
            self._upd('Past drone pad — delivery zone freed for next robot')

        if not self._wps:
            if self._phase == 'stage' and self.busy:
                self._try_enter_delivery()
            return

        PRIORITY = ['amr', 'amr2', 'big_amr']
        tx, ty = self._wps[0]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)

        peers = [p for p in PRIORITY if p != self.key] if self.key in PRIORITY else []
        near_corridor = abs(self.y - SOUTH_Y) < 1.2

        i_delivery_bound = (self._carrying is not None and
                            self._phase in ('stage', 'deliver'))
        if peers and not i_delivery_bound:
            nearest_db = None; nearest_db_d = float('inf')
            with STATE_LOCK:
                for other in peers:
                    if self._peer_delivery_bound(other):
                        od = math.hypot(STATE[other]['x'] - self.x,
                                        STATE[other]['y'] - self.y)
                        if od < nearest_db_d:
                            nearest_db_d = od; nearest_db = other
            if nearest_db is not None and nearest_db_d < self.DELIV_YIELD_DIST:
                self._yield_wait = 1
                self._upd(f'WAIT — {nearest_db} delivering, right of way '
                          f'({nearest_db_d:.1f}m)')
                return
            if self._yield_wait:
                if nearest_db is not None and nearest_db_d < self.DELIV_CLEAR_DIST:
                    self._upd('WAIT — letting delivery robot pass')
                    return
                self._yield_wait = 0
            if self._yield_wait:
                if nearest_db is not None and nearest_db_d < self.DELIV_CLEAR_DIST:
                    self._upd('WAIT — letting delivery robot pass')
                    return
                self._yield_wait = 0

        if peers and i_delivery_bound:
            nx = self.x + (self.STEP * dx / dist if dist > 1e-6 else 0)
            ny = self.y + (self.STEP * dy / dist if dist > 1e-6 else 0)
            with STATE_LOCK:
                for other in peers:
                    ox = STATE[other]['x']; oy = STATE[other]['y']
                    if (math.hypot(ox - nx, oy - ny) < self.OVERLAP_DIST or
                            math.hypot(ox - self.x, oy - self.y) < self.OVERLAP_DIST):
                        blocked = other
                        break
                else:
                    blocked = None
            if blocked is not None:
                self._stuck += 1
                if self._stuck >= self.OVERLAP_STUCK_LIMIT:
                    self._stuck = 0
                    self._forcing = True
                    self._upd(f'UNSTICK — easing past {blocked} (had right of way)')
                else:
                    self._upd(f'WAIT — {blocked} in the way, clearing')
                    return
            else:
                self._stuck = 0

        if peers and not i_delivery_bound:
            proceed_fn = self._proceed_path(tx, ty, dist, dx, dy)
            proceed_score = self._rollout_min_dist(proceed_fn, self.HORIZON, peers)

            if proceed_score < self.SAFE_DIST:
                candidates = [('PROCEED', proceed_fn, proceed_score)]
                hold_fn = self._hold_path()
                candidates.append(('HOLD', hold_fn,
                                    self._rollout_min_dist(hold_fn, self.HORIZON, peers)))
                if near_corridor:
                    yield_fn = self._yield_path()
                    candidates.append(('YIELD', yield_fn,
                                        self._rollout_min_dist(yield_fn, self.HORIZON, peers)))

                i_carrying = self._carrying is not None
                tie_order = {'PROCEED': 0,
                              'HOLD': 1 if i_carrying else 2,
                              'YIELD': 2 if i_carrying else 1}

                my_rank = PRIORITY.index(self.key)
                BIAS = 0.05
                bias = {'PROCEED': 0.0, 'HOLD': 0.0, 'YIELD': 0.0}
                if i_carrying:
                    bias['PROCEED'] += BIAS
                    bias['YIELD']   -= BIAS
                rank_bias = (len(PRIORITY) - 1 - my_rank) * (BIAS / len(PRIORITY))
                bias['PROCEED'] += rank_bias
                bias['YIELD']   -= rank_bias

                best = max(candidates,
                           key=lambda c: (round(c[2] + bias[c[0]], 4), -tie_order[c[0]]))
                name, path_fn, score = best

                if name == 'HOLD':
                    self._stuck += 1
                    if self._stuck >= self.STUCK_LIMIT:
                        if self._try_giveway(peers):
                            self._stuck = 0
                            self._upd('GIVING WAY — backing up to clear the jam')
                            return
                        my_rank = PRIORITY.index(self.key)
                        i_am_top = True
                        with STATE_LOCK:
                            for other in peers:
                                ox = STATE[other]['x']; oy = STATE[other]['y']
                                if (math.hypot(ox - self.x, oy - self.y) < 2.5
                                        and PRIORITY.index(other) < my_rank):
                                    i_am_top = False
                                    break
                        if i_am_top:
                            self._upd(f'UNSTICK — forcing through after standstill')
                            self._stuck = 0
                            self._forcing = True
                        else:
                            self._upd(f'WAIT — peer too close (clearance {score:.2f}m)')
                            return
                    else:
                        self._upd(f'WAIT — peer too close (clearance {score:.2f}m)')
                        return
                elif name == 'YIELD':
                    self._stuck = 0
                    nx2, ny2 = path_fn(1)
                    if math.hypot(nx2 - self.x, ny2 - self.y) > 1e-6:
                        self.yaw = math.atan2(ny2 - self.y, nx2 - self.x)
                        self.x, self.y = nx2, ny2
                        self._place_robot(moving=True)
                    self._upd(f'GIVING WAY — yielding to gap (clearance {score:.2f}m)')
                    return
                else:
                    self._stuck = 0

            if not self._forcing:
                px1, py1 = proceed_fn(1)
                for other in peers:
                    opx, opy = self._predict_peer(other, 1)
                    if math.hypot(px1 - opx, py1 - opy) < 0.4:
                        self._upd(f'WAIT — imminent overlap with {other}')
                        return

        if dist < self.STEP:
            self.x, self.y = tx, ty
            if dx or dy:
                self.yaw = math.atan2(dy, dx)
            self._place_robot(moving=False)
            self._wps.pop(0)
            if not self._wps:
                self._segment_done()
            return

        self._stuck = 0
        self.yaw = math.atan2(dy, dx)
        self.x += self.STEP * dx / dist
        self.y += self.STEP * dy / dist
        self._place_robot(moving=True)

    def _segment_done(self):
        if self._phase == 'out':
            self._upd(f'PICK {self._box_name}')
            self._carrying = self._box_name
            self._pose_cli.call_async(
                self._pose_req(self._box_name, self.x, self.y,
                               self.carry_z, self.yaw))
            time.sleep(0.4)

            
            with STATE_LOCK:
                if STATE.get('delivery_occupied') is None:
                    STATE['delivery_occupied'] = self.key
                    acquired_now = True
                else:
                    acquired_now = False

            if acquired_now:
                self._released_mutex = False
                self._wps = self._route_direct_to_drop(self.drop_xy)
                self._phase = 'deliver'
                self._upd('Delivery zone free — heading straight to place')
            else:
                with STATE_LOCK:
                    if self.key not in STATE['delivery_waiters']:
                        STATE['delivery_waiters'][self.key] = self._stage_order()
                self._wps = self._route_to_stage(HOLD_Y)
                self._phase = 'stage'
                self._upd('Carrying to delivery — heading to staging lane')
        elif self._phase == 'stage':
            self._try_enter_delivery()
        elif self._phase == 'deliver':
            self._upd('PLACE box at delivery')
            self._pose_cli.call_async(
                self._pose_req(self._box_name,
                               self.drop_xy[0], self.drop_xy[1],
                               self.drop_z, 0.0))
            self._carrying = None
            time.sleep(0.3)
            self._wps = self._route_home()
            self._phase = 'home'
            self._upd('Returning home')
        else:
            self._upd('Parked at home')
            self.busy = False
            with STATE_LOCK:
                if STATE.get('delivery_occupied') == self.key:
                    STATE['delivery_occupied'] = None
                STATE['done'].append(self._box_name)
                STATE[self.key]['status'] = 'IDLE'
                STATE[self.key]['job'] = None
            cb = self._done_cb; self._done_cb = None
            if cb: cb(self.key)

    @staticmethod
    def _stage_order():
        c = STATE.get('_stage_counter', 0) + 1
        STATE['_stage_counter'] = c
        return c

    def _try_enter_delivery(self):
        with STATE_LOCK:
            waiters = STATE['delivery_waiters']
            if self.key not in waiters:
                waiters[self.key] = self._stage_order()
            occ = STATE.get('delivery_occupied')
            order = sorted(waiters, key=lambda k: waiters[k])
            my_slot = order.index(self.key)
            front = (my_slot == 0)
            if (occ is None or occ == self.key) and front:
                STATE['delivery_occupied'] = self.key
                del STATE['delivery_waiters'][self.key]
                acquired = True
            else:
                acquired = False

        if acquired:
            self._released_mutex = False
            self._wps = self._route_stage_to_drop(self.drop_xy)
            self._phase = 'deliver'
            self._upd('Entering delivery zone (front of queue)')
        else:
            slot_y = HOLD_Y + my_slot * STAGE_SLOT_DY
            slot_y = min(slot_y, MID_Y)
            if abs(self.x - STAGE_X) > 0.1 or abs(self.y - slot_y) > 0.1:
                self._wps = [(STAGE_X, slot_y)]
                self._phase = 'stage'
            else:
                self._wps = []
            occ_now = STATE.get('delivery_occupied')
            if occ_now and occ_now != self.key:
                self._upd(f'WAIT — slot {my_slot}, delivery zone busy ({occ_now})')
            else:
                self._upd(f'WAIT — slot {my_slot} in staging queue')

    def _route_to_stage(self, stage_y):
        fx, fy = self.x, self.y
        aisle = min(V_AISLES, key=lambda a: (round(abs(a - fx), 4),
                                              abs(a - STAGE_X)))
        wps = []
        if fy >= NORTH_CORR_Y:
            if abs(fx - aisle) > 0.3:
                wps.append((aisle, fy))
            wps.append((aisle, MID_Y))
            wps.append((STAGE_X, MID_Y))
            wps.append((STAGE_X, stage_y))
        else:
            start_corr = self._corridor_for(fy)
            if abs(fy - start_corr) > 0.3:
                wps.append((fx, start_corr))
            if abs(fx - aisle) > 0.3:
                wps.append((aisle, start_corr))
            wps.append((STAGE_X, start_corr))
            wps.append((STAGE_X, stage_y))
        return wps

    def _route_direct_to_drop(self, drop):
        """Direct route straight to delivery — used when the delivery
        zone is free at pickup time, so we skip the STAGE_X/HOLD_Y
        detour entirely. Still exits to a clear aisle first (safe for
        interior picks), then crosses the south corridor to the drop."""
        fx, fy = self.x, self.y
        dx, dy = drop
        aisle = min(V_AISLES, key=lambda a: abs(a - fx))
        start_corr = self._corridor_for(fy)
        wps = []
        if abs(fy - start_corr) > 0.3:
            wps.append((fx, start_corr))
        if abs(fx - aisle) > 0.3:
            wps.append((aisle, start_corr))
        wps.append((aisle, SOUTH_LANE_OUT))
        wps.append((dx, SOUTH_LANE_OUT))
        wps.append((dx, dy))
        return wps

    def _route_stage_to_drop(self, drop):
        dx, dy = drop
        wps = []
        wps.append((STAGE_X, SOUTH_LANE_OUT))
        wps.append((dx, SOUTH_LANE_OUT))
        wps.append((dx, dy))
        return wps

    def _route_home(self):
        fx, fy = self.x, self.y
        hx, hy = self.home
        wps = []
        if abs(fy - SOUTH_LANE_RETURN) > 0.3:
            wps.append((fx, SOUTH_LANE_RETURN))
        wps.append((hx, SOUTH_LANE_RETURN))
        wps.append((hx, hy))
        return wps


# ═══════════════════════════ SCHEDULER ═══════════════════════
class Scheduler:
    def __init__(self, drone, amr, amr2, big):
        self.r = {'drone': drone, 'amr': amr, 'amr2': amr2, 'big_amr': big}
        self._heap = []
        self._n = itertools.count()
        self._lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def add(self, box, prio='normal'):
        if box not in ALL_BOXES:
            return False, 'unknown box'
        robot = robot_for(box)
        with self._lock:
            heapq.heappush(self._heap,
                           (PRIO.get(prio,1), next(self._n), box, robot))
        self._sync_queue()
        log(f"[ORDER] {box} -> {robot.upper()} [{prio.upper()}]")
        return True, robot

    def _sync_queue(self):
        with self._lock:
            q = sorted(self._heap)
        with STATE_LOCK:
            STATE['queue'] = [
                {'box': b, 'robot': r,
                 'priority': {0:'urgent',1:'normal',2:'low'}[p]}
                for (p, n, b, r) in q]

    def _loop(self):
        while True:
            time.sleep(0.4)
            with self._lock:
                q = sorted(self._heap)
            assigned = False
            for item in q:
                p, n, box, robot = item
                ctl = self.r[robot]
                if not ctl.busy:
                    with self._lock:
                        self._heap.remove(item)
                        heapq.heapify(self._heap)
                    self._sync_queue()
                    ctl.run_job(box, self._on_done)
                    assigned = True
                    break
            if not assigned:
                time.sleep(0.1)

    def _on_done(self, robot):
        log(f"[SCHEDULER] {robot.upper()} job complete")
        self._sync_queue()


SCHED = None

# ═══════════════════════════ WEBSOCKET ═══════════════════════
CLIENTS = set()

async def handler(ws):
    CLIENTS.add(ws)
    try:
        await ws.send(json.dumps({'type':'state','data':STATE,
                                  'boxes': sorted(ALL_BOXES.keys())}))
        async for raw in ws:
            m = json.loads(raw)
            if m.get('type') == 'order':
                ok, info = SCHED.add(m.get('box',''),
                                     m.get('priority','normal'))
            elif m.get('type') == 'auto':
                for b, p in [('box_2_2_high_la','urgent'),
                             ('box_3_1_low_ra','normal'),
                             ('bigbox_1_2_ll','normal'),
                             ('box_2_2_low_la','normal'),
                             ('box_3_3_high_rb','low')]:
                    SCHED.add(b, p)
            elif m.get('type') == 'speed':
                global SPEED_MULT
                try:
                    v = float(m.get('value', 1.0))
                    SPEED_MULT = max(0.25, min(4.0, v))
                    log(f"[SIM] speed set to {SPEED_MULT:.2f}x")
                except (TypeError, ValueError):
                    pass
    finally:
        CLIENTS.discard(ws)

async def broadcaster():
    while True:
        await asyncio.sleep(0.4)
        if CLIENTS:
            with STATE_LOCK:
                msg = json.dumps({'type':'state','data':STATE})
            await asyncio.gather(*[c.send(msg) for c in CLIENTS],
                                 return_exceptions=True)

async def ws_main():
    async with websockets.serve(handler, 'localhost', 8765):
        log('UI server: ws://localhost:8765 — open warehouse_ui.html')
        await broadcaster()


# ═══════════════════════════ MAIN ════════════════════════════
def main():
    global SCHED
    rclpy.init()
    drone = DroneCtl()
    amr   = AmrCtl('amr', '/warehouse_amr/cmd_vel',
                   '/model/warehouse_amr/odometry',
                   AMR_HOME, AMR_CARRY_Z, AMR_DROP, AMR_BOX_DROP_Z,
                   spawn_yaw=1.5708, model_name='warehouse_amr')
    amr2  = AmrCtl('amr2', '/warehouse_amr2/cmd_vel',
                   '/model/warehouse_amr2/odometry',
                   AMR2_HOME, AMR_CARRY_Z, AMR2_DROP, AMR_BOX_DROP_Z,
                   spawn_yaw=1.5708, model_name='warehouse_amr_2')
    big   = AmrCtl('big_amr', '/big_amr/cmd_vel',
                   '/model/big_amr/odometry',
                   BIG_HOME, BIG_CARRY_Z, BIG_DROP, BIG_BOX_DROP_Z,
                   spawn_yaw=-1.5708, model_name='big_amr')
    SCHED = Scheduler(drone, amr, amr2, big)

    ex = MultiThreadedExecutor()
    for n in (drone, amr, amr2, big): ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    try:
        asyncio.run(ws_main())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()