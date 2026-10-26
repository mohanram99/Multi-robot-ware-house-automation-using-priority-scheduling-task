#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class PIDController:
    def __init__(self, kp, ki, kd, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.limit, min(self.limit, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


def quat_to_yaw(qx, qy, qz, qw):
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class DronePositionController(Node):
    def __init__(self):
        super().__init__('drone_position_controller')

        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/drone/odometry', self.odom_cb, 10)

        # PID controllers — world frame errors
        self.pid_x = PIDController(kp=0.8, ki=0.0, kd=0.3, limit=4.0) #changed 24.6.26 speed
        self.pid_y = PIDController(kp=0.8, ki=0.0, kd=0.3, limit=4.0)
        self.pid_z = PIDController(kp=1.0, ki=0.0, kd=0.4, limit=2.0)

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.goal_active = False
        self.goal_reached_cb = None

        self.POSITION_THRESHOLD = 0.35  # metres

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.control_loop)  

        self.get_logger().info('Drone Position Controller ready.')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.current_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def set_goal(self, x, y, z, reached_cb=None):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.goal_active = True
        self.goal_reached_cb = reached_cb
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.get_logger().info(f'New goal: ({x:.2f}, {y:.2f}, {z:.2f})')

    def stop(self):
        self.goal_active = False
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if not self.goal_active:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0:
            return

        # World-frame errors
        ex_world = self.target_x - self.current_x
        ey_world = self.target_y - self.current_y
        ez       = self.target_z - self.current_z

        distance = math.sqrt(ex_world**2 + ey_world**2 + ez**2)

        if distance < self.POSITION_THRESHOLD:
            self.get_logger().info(
                f'Goal reached: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})')
            self.stop()
            if self.goal_reached_cb:
                self.goal_reached_cb()
            return

        # PID on world-frame errors
        vx_world = self.pid_x.compute(ex_world, dt)
        vy_world = self.pid_y.compute(ey_world, dt)
        vz       = self.pid_z.compute(ez, dt)

        # Rotate world-frame velocity → drone body frame using current yaw
        yaw = self.current_yaw
        vx_body =  vx_world * math.cos(yaw) + vy_world * math.sin(yaw)
        vy_body = -vx_world * math.sin(yaw) + vy_world * math.cos(yaw)

        twist = Twist()
        twist.linear.x = vx_body
        twist.linear.y = vy_body
        twist.linear.z = vz

        self.cmd_pub.publish(twist)
        self.get_logger().debug(
            f'pos=({self.current_x:.2f},{self.current_y:.2f},{self.current_z:.2f}) '
            f'err=({ex_world:.2f},{ey_world:.2f},{ez:.2f}) '
            f'yaw={math.degrees(yaw):.1f}deg '
            f'cmd=({vx_body:.2f},{vy_body:.2f},{vz:.2f})')

