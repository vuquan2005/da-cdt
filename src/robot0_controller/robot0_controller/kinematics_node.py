#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mecanum Base Kinematics Node for Robot0.
Translates planar Twist commands (/cmd_vel) into individual wheel angular velocities (rad/s)
for 4WD Mecanum wheel mobile base.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Robot0KinematicsNode(Node):
    def __init__(self):
        super().__init__('robot0_kinematics')

        # Declare Parameters
        self.declare_parameter('wheel_radius', 0.0487)       # Wheel radius (m)
        self.declare_parameter('wheelbase_lx', 0.1000)       # Half wheelbase: front-rear / 2 (m)
        self.declare_parameter('track_ly', 0.1539)           # Half track width: left-right / 2 (m)
        self.declare_parameter('cmd_timeout_sec', 0.5)       # Timeout to stop wheels if no cmd_vel
        self.declare_parameter('rate_hz', 50.0)              # Control loop frequency (Hz)

        # Retrieve parameter values
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.lx = float(self.get_parameter('wheelbase_lx').value)
        self.ly = float(self.get_parameter('track_ly').value)
        self.k_geom = self.lx + self.ly
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        # Publishers for 4 Mecanum wheels
        self.wheel_fl_pub = self.create_publisher(Float64, '/wheel_fl_cmd_vel', 10)
        self.wheel_fr_pub = self.create_publisher(Float64, '/wheel_fr_cmd_vel', 10)
        self.wheel_rl_pub = self.create_publisher(Float64, '/wheel_rl_cmd_vel', 10)
        self.wheel_rr_pub = self.create_publisher(Float64, '/wheel_rr_cmd_vel', 10)

        # Subscriber for Twist velocity command
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # State tracking
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.last_cmd_time = 0.0
        self.is_moving = False

        # Periodic control loop timer
        timer_period = 1.0 / rate_hz if rate_hz > 0 else 0.02
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f'Robot0 Mecanum Kinematics Node started (r={self.wheel_radius:.4f}m, '
            f'lx={self.lx:.4f}m, ly={self.ly:.4f}m, k={self.k_geom:.4f}m, rate={rate_hz:.1f}Hz).'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Callback triggered when a new Twist message is received on /cmd_vel."""
        self.target_vx = float(msg.linear.x)
        self.target_vy = float(msg.linear.y)
        self.target_wz = float(msg.angular.z)
        self.last_cmd_time = time.time()

    def publish_wheels(self, w_fl: float, w_fr: float, w_rl: float, w_rr: float):
        """Publishes angular velocity commands to all four wheels."""
        msg = Float64()
        msg.data = float(w_fl)
        self.wheel_fl_pub.publish(msg)

        msg.data = float(w_fr)
        self.wheel_fr_pub.publish(msg)

        msg.data = float(w_rl)
        self.wheel_rl_pub.publish(msg)

        msg.data = float(w_rr)
        self.wheel_rr_pub.publish(msg)

    def control_loop(self):
        """Main control loop calculating Mecanum inverse kinematics."""
        now = time.time()
        time_since_last_cmd = now - self.last_cmd_time

        # Check if command has timed out or is near zero
        is_zero_cmd = (
            abs(self.target_vx) < 1e-4 and
            abs(self.target_vy) < 1e-4 and
            abs(self.target_wz) < 1e-4
        )

        if time_since_last_cmd > self.cmd_timeout or is_zero_cmd:
            if self.is_moving:
                self.publish_wheels(0.0, 0.0, 0.0, 0.0)
                self.is_moving = False
            return

        # -------------------------------------------------------------
        # Mecanum Inverse Kinematics Formula
        # w_fl = (vx - vy - (lx + ly) * wz) / r
        # w_fr = (vx + vy + (lx + ly) * wz) / r
        # w_rl = (vx + vy - (lx + ly) * wz) / r
        # w_rr = (vx - vy + (lx + ly) * wz) / r
        # -------------------------------------------------------------
        w_fl = (self.target_vx - self.target_vy - self.k_geom * self.target_wz) / self.wheel_radius
        w_fr = (self.target_vx + self.target_vy + self.k_geom * self.target_wz) / self.wheel_radius
        w_rl = (self.target_vx + self.target_vy - self.k_geom * self.target_wz) / self.wheel_radius
        w_rr = (self.target_vx - self.target_vy + self.k_geom * self.target_wz) / self.wheel_radius

        self.publish_wheels(w_fl, w_fr, w_rl, w_rr)
        self.is_moving = True


def main(args=None):
    rclpy.init(args=args)
    node = Robot0KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_wheels(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
