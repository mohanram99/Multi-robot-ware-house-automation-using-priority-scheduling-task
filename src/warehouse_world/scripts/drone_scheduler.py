#!/usr/bin/env python3
"""
Drone Scheduler for high boxes 
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose
import math, threading, sys, os
import subprocess
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drone_position_controller import DronePositionController

# ── DECLARATION ─────────────────────────────────────────────────
WORLD_NAME     = "multi_robot_warehouse"   
CARRY_OFFSET   = 0.26   

DRONE_PAD_X = 0.0
DRONE_PAD_Y = -10.0
DRONE_PAD_Z = 0.75
CRUISE_ALT  = 3.0       # height to clear shelves
RETURN_ALT  = 1.3       # low fast return 

DROPOFF_X   = 10.0
DROPOFF_Y   = -10.0
DROPOFF_Z   = 0.5      

DELIVERY_BOX_Z = 0.16   

NORMAL_H    = 0.26
CLEARANCE   = 0.08
FRAME_TOP_Z = 2       # frames 1.53m tall - 5.5.26

THRESHOLD_NORMAL = 0.35
THRESHOLD_SHELF  = 0.25

def hz(box_z, box_h):
    return round(box_z + box_h/2 + CLEARANCE, 3)

# ── BOX MAP — HIGH BOARD ONLY ─────────────────────────────────
BOX_MAP = {
    # ROW 2 — HIGH BOARD — TOP-DOWN
    'box_2_1_high_la':{'x':-9.45,'y':-0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_1_high_lb':{'x':-9.45,'y': 0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_1_high_ra':{'x':-8.55,'y':-0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_1_high_rb':{'x':-8.55,'y': 0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_2_high_la':{'x':-3.45,'y':-0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_2_high_lb':{'x':-3.45,'y': 0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_2_high_ra':{'x':-2.55,'y':-0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_2_high_rb':{'x':-2.55,'y': 0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_3_high_la':{'x': 2.55,'y':-0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_3_high_lb':{'x': 2.55,'y': 0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_3_high_ra':{'x': 3.45,'y':-0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_3_high_rb':{'x': 3.45,'y': 0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_4_high_la':{'x': 8.55,'y':-0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_4_high_lb':{'x': 8.55,'y': 0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_4_high_ra':{'x': 9.45,'y':-0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_2_4_high_rb':{'x': 9.45,'y': 0.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    # ROW 3 — HIGH BOARD — TOP-DOWN
    'box_3_1_high_la':{'x':-9.45,'y':-6.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_1_high_lb':{'x':-9.45,'y':-5.55,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_1_high_ra':{'x':-8.55,'y':-6.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_1_high_rb':{'x':-8.55,'y':-5.55,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_2_high_la':{'x':-3.45,'y':-6.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_2_high_lb':{'x':-3.45,'y':-5.55,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_2_high_ra':{'x':-2.55,'y':-6.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_2_high_rb':{'x':-2.55,'y':-5.55,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_3_high_la':{'x': 2.55,'y':-6.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_3_high_lb':{'x': 2.55,'y':-5.55,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_3_high_ra':{'x': 3.45,'y':-6.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_3_high_rb':{'x': 3.45,'y':-5.55,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_4_high_la':{'x': 8.55,'y':-6.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_4_high_lb':{'x': 8.55,'y':-5.55,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_4_high_ra':{'x': 9.45,'y':-6.45,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
    'box_3_4_high_rb':{'x': 9.45,'y':-5.55,'hover_z':hz(1.655,NORMAL_H),'top_down':True},
}


class DroneScheduler(DronePositionController):
    def __init__(self):
        super().__init__()
        self.enable_pub = self.create_publisher(Bool, '/drone/enable', 10)
        # Low-latency pose service client (bridged from Gazebo)
        self._pose_cli = self.create_client(
            SetEntityPose, f'/world/{WORLD_NAME}/set_pose')
        self._armed    = False
        self._busy     = False
        self._queue    = []
        self._current  = None
        self._box      = None
        self._carrying = None
        
        self._carry_thread = threading.Thread(
            target=self._carry_loop, daemon=True)
        self._carry_thread.start()
        #self.create_timer(1.0, self._arm_once)
        self._arm_timer = self.create_timer(1.0, self._arm_once)
        self.get_logger().info('Drone Scheduler v9 ready.')

    # ──  ───────────────────────────────────────────────

    def _reset_motion_params(self):
        self.POSITION_THRESHOLD = THRESHOLD_NORMAL
        self.pid_x.limit = 1.0
        self.pid_y.limit = 1.0
        self.pid_z.limit = 1.5

    def _one_shot(self, delay, cb):
        """Fire cb once after delay seconds, then cancel the timer."""
        def _f():
            _t.cancel()
            cb()
        _t = self.create_timer(delay, _f)

    def _arm_once(self):
        if not self._armed:
            msg = Bool(); msg.data = True
            self.enable_pub.publish(msg)
            self._armed = True
            self.get_logger().info('Drone armed.')
        self._arm_timer.cancel()  

    def _make_pose_request(self, box_name, x, y, z):
        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = box_name
        req.entity.type = Entity.MODEL
        req.pose = Pose()
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.w = 1.0
        return req

    def _set_box_pose_checked(self, box_name, x, y, z):
        """One-off teleport via CLI with visible errors (attach/detach)."""
        req = (f'name: "{box_name}", '
               f'position: {{x: {x}, y: {y}, z: {z}}}')
        result = subprocess.run([
            'gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '300', '--req', req],
            capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(
                f'BOX TELEPORT FAILED for {box_name}: {result.stderr.strip()} '
                f'— check WORLD_NAME matches `gz service -l | grep set_pose`')
            return False
        self.get_logger().info(f'Box teleport OK: {box_name}')
        return True

    def _carry_loop(self):
        """Low-latency box follow via bridged ROS2 set_pose service."""
        while not self._pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pose service bridge...')
        self.get_logger().info('set_pose service connected — carry ready.')
        while True:
            box = self._carrying
            if box:
                req = self._make_pose_request(
                    box,
                    self.current_x,
                    self.current_y,
                    self.current_z - CARRY_OFFSET)
                self._pose_cli.call_async(req)   # ~ms latency, ordered
                time.sleep(0.03)                  # ~33 Hz updates
            else:
                time.sleep(0.05)

    # ── PUBLIC API ────────────────────────────────────────────

    def schedule(self, box_name):
        if box_name not in BOX_MAP:
            self.get_logger().error(f'Unknown box: {box_name}')
            return False
        self._queue.append(box_name)
        self.get_logger().info(f'Queued: {box_name} (queue: {len(self._queue)})')
        if not self._busy:
            self._next_job()
        return True

    def status(self):
        if self._busy:    return f'BUSY — picking {self._current}'
        elif self._queue: return f'IDLE — {len(self._queue)} pending'
        else:             return 'IDLE — ready'

    # ── JOB LIFECYCLE ─────────────────────────────────────────

    def _next_job(self):
        if not self._queue:
            self._busy = False
            self.get_logger().info('All jobs done.')
            return
        self._busy    = True
        self._current = self._queue.pop(0)
        self._box     = BOX_MAP[self._current]
        self._reset_motion_params()
        self.get_logger().info(f'=== JOB: {self._current} === TOP-DOWN')
        self._step_ascend()

    # ── MISSION STEPS ─────────────────────────────────────────

    def _step_ascend(self):
        self._reset_motion_params()
        msg = Bool(); msg.data = True          
        self.enable_pub.publish(msg)
        self._armed = True
        self.get_logger().info('1: Ascending...')
        self.set_goal(DRONE_PAD_X, DRONE_PAD_Y, CRUISE_ALT,
                      reached_cb=self._step_fly_to_box_xy)

    def _step_fly_to_box_xy(self):
        self._reset_motion_params()
        b = self._box
        self.get_logger().info(f'2: Flying above box ({b["x"]}, {b["y"]}) at {CRUISE_ALT}m...')
        self.set_goal(b['x'], b['y'], CRUISE_ALT,
                      reached_cb=self._step_descend_to_frame_top)

    def _step_descend_to_frame_top(self):
        self._reset_motion_params()
        b = self._box
        #self.pid_z.limit = 0.8              
        #self.POSITION_THRESHOLD = THRESHOLD_SHELF
        self.get_logger().info(f'3: Descending to just above frame top ({FRAME_TOP_Z}m)...')
        self.set_goal(b['x'], b['y'], FRAME_TOP_Z,
                      reached_cb=self._step_descend_to_box)

    def _step_descend_to_box(self):
        self._reset_motion_params()
        b = self._box
        self.pid_z.limit = 0.3   # slow final descent
        #self.POSITION_THRESHOLD = THRESHOLD_SHELF 
        self.get_logger().info(f'4: Slow descent to box (hover_z={b["hover_z"]})...')
        self.set_goal(b['x'], b['y'], b['hover_z'],
                      reached_cb=self._step_settle_at_box)
        
    def _step_settle_at_box(self):
        
        self.get_logger().info('4b: Settling at box...')
        self._one_shot(1.5, self._step_attach)

    def _step_attach(self):
        self._reset_motion_params()
        self.get_logger().info(f'5: ATTACH {self._current} — box lifted')
        
        ok = self._set_box_pose_checked(
            self._current,
            self.current_x, self.current_y,
            self.current_z - CARRY_OFFSET)
        if ok:
            self._carrying = self._current   # carry loop takes over
        else:
            self.get_logger().error('Attach failed — aborting job.')
            self._job_done()
            return
        self._one_shot(1.0, self._step_ascend_with_box)

    def _step_ascend_with_box(self):
        self._reset_motion_params()
        b = self._box
        self.get_logger().info('6: Ascending with box...')
        self.set_goal(b['x'], b['y'], CRUISE_ALT,
                      reached_cb=self._step_fly_to_dropoff)

    def _step_fly_to_dropoff(self):
        self._reset_motion_params()
        self.get_logger().info('7: Flying to delivery station...')
        self.set_goal(DROPOFF_X, DROPOFF_Y, CRUISE_ALT,
                      reached_cb=self._step_descend_drop)

    def _step_descend_drop(self):
        self._reset_motion_params()
          
        #self.POSITION_THRESHOLD = THRESHOLD_SHELF
        self.get_logger().info('8: Descending to delivery station...')
        self.set_goal(DROPOFF_X, DROPOFF_Y, DROPOFF_Z,
                      reached_cb=self._step_detach)

    def _step_detach(self):
        self._reset_motion_params()
        self.get_logger().info('9: DETACH — placing box on delivery mat')
        self._carrying = None                    
        time.sleep(0.1)                          
        self._set_box_pose_checked(              # box sits on the mat
            self._current, DROPOFF_X, DROPOFF_Y, DELIVERY_BOX_Z)
        #self._one_shot(1.0, self._step_return_ascend)
        self._step_return_ascend()

    def _step_return_ascend(self):
        self._reset_motion_params()
                      
        #self.POSITION_THRESHOLD = THRESHOLD_SHELF
        self.get_logger().info('10: Ascending to return altitude...')
        self.set_goal(DROPOFF_X, DROPOFF_Y, RETURN_ALT,
                      reached_cb=self._step_fly_to_pad)

    def _step_fly_to_pad(self):
        self._reset_motion_params()
        self.get_logger().info('11: Flying to pad (low return)...')
        self.set_goal(DRONE_PAD_X, DRONE_PAD_Y, RETURN_ALT,
                      reached_cb=self._step_final_land)

    def _step_final_land(self):
        self._reset_motion_params()
        self.pid_z.limit = 0.3
        self.POSITION_THRESHOLD = 0.20      
        self.get_logger().info('12: Descending onto pad...')
        self.set_goal(DRONE_PAD_X, DRONE_PAD_Y, DRONE_PAD_Z,
                      reached_cb=self._settle_on_pad)

    def _settle_on_pad(self):
        self.stop()
        msg = Bool(); msg.data = False        
        self.enable_pub.publish(msg)
        self._armed = False
        self.get_logger().info('Motors off — drone parked on pad.')
        self._job_done()

    def _job_done(self):
        self.get_logger().info(f'=== DONE: {self._current} — delivered ===')
        self._current = None
        self._next_job()


# ── CLI ───────────────────────────────────────────────────────

def print_menu():
    print("\n" + "="*58)
    print("  WAREHOUSE DRONE SCHEDULER ") #not using this
    print("  HIGH BOARD BOXES ONLY — TOP-DOWN pick")
    print("="*58)
    print("""
  Path: Ascend → fly to box XY → descend → attach → ascend
        → delivery (box on mat) → LOW fast return → land (motors off)

  BOXES:
    Row 2: box_2_<1-4>_high_<la|lb|ra|rb>
    Row 3: box_3_<1-4>_high_<la|lb|ra|rb>

    la = left-front   lb = left-back
    ra = right-front  rb = right-back
    col: 1(x=-9) 2(x=-3) 3(x=3) 4(x=9)

  Commands:
    pick <name>   e.g. pick box_2_2_high_la
    list          show all boxes
    list 2        row 2 only
    list 3        row 3 only
    status
    quit
""")


def list_boxes(row=None):
    for name in sorted(BOX_MAP.keys()):
        r = name[4]
        if row and r != row:
            continue
        b = BOX_MAP[name]
        print(f"  {name:28s}  x={b['x']:6.2f}  y={b['y']:6.2f}  hover_z={b['hover_z']:.3f}")


def main():
    rclpy.init()
    node = DroneScheduler()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    print_menu()
    try:
        while True:
            try: cmd = input("  > ").strip()
            except EOFError: break
            if not cmd: continue
            parts = cmd.split()
            v = parts[0].lower()
            if v in ('quit', 'exit'): break
            elif v == 'status': print(f"  {node.status()}")
            elif v == 'list':
                row = parts[1] if len(parts) > 1 else None
                list_boxes(row)
            elif v == 'pick':
                if len(parts) < 2: print("  Usage: pick <box_name>")
                elif node.schedule(parts[1]): print(f"  Scheduled: {parts[1]}")
                else: print("  Unknown box. Type 'list' to see all.")
            else:
                print("  Commands: pick, list, status, quit")
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()