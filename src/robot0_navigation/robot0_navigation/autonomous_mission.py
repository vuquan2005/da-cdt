#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Autonomous Pallet Retrieval & Drop-Off Mission Node for Robot0.
Refactored into robot0_navigation package using standardized ROS 2 architecture.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from robot0_navigation.arena_coordinates import (
    ROBOT_SPAWN,
    STORAGE_RACKS,
    PALLETS,
    DROPOFF_ZONES,
    LIFT_HEIGHT_TRANSIT,
    LIFT_HEIGHT_LEVEL1_INSERT,
    LIFT_HEIGHT_LEVEL1_CARRY,
    LIFT_HEIGHT_LEVEL2_INSERT,
    LIFT_HEIGHT_LEVEL2_CARRY,
    LIFT_ARM_LATERAL_OFFSET,
    get_dropoff_by_color,
)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class AutonomousPalletMission(Node):
    # FSM State definitions
    STATE_WAIT_ODOM = "WAIT_ODOM"
    STATE_NAV_STAGING = "NAV_STAGING"
    STATE_PRE_LIFT_ALIGN = "PRE_LIFT_ALIGN"
    STATE_FORK_INSERT = "FORK_INSERT"
    STATE_LIFT_PALLET = "LIFT_PALLET"
    STATE_FORK_RETRACT = "FORK_RETRACT"
    STATE_NAV_DELIVERY = "NAV_DELIVERY"
    STATE_LOWER_PALLET = "LOWER_PALLET"
    STATE_COMPLETED = "COMPLETED"

    def __init__(self):
        super().__init__('autonomous_pallet_mission')

        # Declare ROS 2 parameters
        self.declare_parameter('target_rack', 'rack_left_bot')
        self.declare_parameter('target_shelf_level', 1)  # 1 for Bottom shelf, 2 for Top shelf
        self.declare_parameter('target_slot', 'left')     # 'left' or 'right'
        self.declare_parameter('dropoff_color', '')       # Empty for return home, or 'blue', 'green', etc.

        self.rack_name = self.get_parameter('target_rack').get_parameter_value().string_value
        self.shelf_level = self.get_parameter('target_shelf_level').get_parameter_value().integer_value
        self.slot = self.get_parameter('target_slot').get_parameter_value().string_value.lower()
        self.dropoff_color = self.get_parameter('dropoff_color').get_parameter_value().string_value.lower()

        # Mecanum Robot Geometry
        self.wheel_radius = 0.0487
        self.lx = 0.1000
        self.ly = 0.1539
        self.k_geom = self.lx + self.ly

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/lift_joint_cmd', 10)
        self.wheel_fl_pub = self.create_publisher(Float64, '/wheel_fl_cmd_vel', 10)
        self.wheel_fr_pub = self.create_publisher(Float64, '/wheel_fr_cmd_vel', 10)
        self.wheel_rl_pub = self.create_publisher(Float64, '/wheel_rl_cmd_vel', 10)
        self.wheel_rr_pub = self.create_publisher(Float64, '/wheel_rr_cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Internal Robot Pose
        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.home_x: Optional[float] = None
        self.home_y: Optional[float] = None
        self.home_yaw: Optional[float] = None

        # Mission State
        self.state = self.STATE_WAIT_ODOM
        self.state_start_time = time.time()

        # Target Coordinates & Waypoint Calculation
        self._calculate_target_geometry()

        # Control Loop Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("=================================================================")
        self.get_logger().info(f"Robot0 Navigation Mission Initialized:")
        self.get_logger().info(f"  Rack: {self.rack_name} | Shelf Level: {self.shelf_level} | Slot: {self.slot.upper()}")
        self.get_logger().info(f"  Staging Pose : X={self.staging_x:.3f}m, Y={self.staging_y:.3f}m, Yaw={math.degrees(self.target_yaw):.1f}°")
        self.get_logger().info(f"  Insertion Pose: X={self.insert_x:.3f}m, Y={self.insert_y:.3f}m")
        self.get_logger().info(f"  Lift Heights  : Insert={self.lift_insert_height*100:.2f}cm, Carry={self.lift_carry_height*100:.2f}cm")
        if self.dropoff_target:
            self.get_logger().info(f"  Drop-off Zone : {self.dropoff_target.description}")
        else:
            self.get_logger().info(f"  Delivery Mode : Return to Home Base")
        self.get_logger().info("=================================================================")

    def _calculate_target_geometry(self):
        # 1. Look up Rack
        if self.rack_name not in STORAGE_RACKS:
            self.get_logger().warn(f"Rack '{self.rack_name}' not found. Defaulting to 'rack_left_bot'")
            self.rack_name = "rack_left_bot"
        rack = STORAGE_RACKS[self.rack_name]

        # 2. Lift Height selection
        if self.shelf_level == 2:
            self.lift_insert_height = LIFT_HEIGHT_LEVEL2_INSERT
            self.lift_carry_height = LIFT_HEIGHT_LEVEL2_CARRY
        else:
            self.lift_insert_height = LIFT_HEIGHT_LEVEL1_INSERT
            self.lift_carry_height = LIFT_HEIGHT_LEVEL1_CARRY

        # 3. Calculate target alignment based on rack orientation
        # For left racks (Yaw = pi/2): slot offset is in world Y
        if "left" in self.rack_name:
            self.target_yaw = math.pi  # Facing towards left rack (-X)
            # Lateral alignment with kinematic offset
            if self.rack_name == "rack_left_bot":
                base_y = 0.5810 if self.slot == 'left' else 0.7010
            elif self.rack_name == "rack_left_mid":
                base_y = -0.0660 if self.slot == 'left' else 0.0540
            else:  # rack_left_top
                base_y = -0.7090 if self.slot == 'left' else -0.5890
            
            self.staging_x = -1.450
            self.insert_x = -1.685
            self.staging_y = base_y - LIFT_ARM_LATERAL_OFFSET
            self.insert_y = self.staging_y
        else:
            # Bottom rack (rack_bot_mid_left, Yaw = 0.0): facing world +Y
            self.target_yaw = math.pi / 2.0
            base_x = -0.550 if self.slot == 'left' else -0.430
            self.staging_x = base_x + LIFT_ARM_LATERAL_OFFSET
            self.insert_x = self.staging_x
            self.staging_y = 0.450
            self.insert_y = 0.685

        # 4. Drop-off target
        self.dropoff_target = get_dropoff_by_color(self.dropoff_color) if self.dropoff_color else None

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_x = pos.x
        self.current_y = pos.y
        self.current_yaw = quat_to_yaw(ori.x, ori.y, ori.z, ori.w)

        if self.home_x is None:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_yaw = self.current_yaw
            self.get_logger().info(
                f"Spawn Pose Locked: X={self.home_x:.3f}m, Y={self.home_y:.3f}m, Yaw={math.degrees(self.home_yaw):.1f}°"
            )

    def publish_twist(self, vx: float, vy: float, wz: float):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(twist)

        # Synchronize wheel velocity commands
        w_fl = (vx - vy - self.k_geom * wz) / self.wheel_radius
        w_fr = (vx + vy + self.k_geom * wz) / self.wheel_radius
        w_rl = (vx + vy - self.k_geom * wz) / self.wheel_radius
        w_rr = (vx - vy + self.k_geom * wz) / self.wheel_radius

        m = Float64()
        m.data = float(w_fl); self.wheel_fl_pub.publish(m)
        m.data = float(w_fr); self.wheel_fr_pub.publish(m)
        m.data = float(w_rl); self.wheel_rl_pub.publish(m)
        m.data = float(w_rr); self.wheel_rr_pub.publish(m)

    def set_lift(self, height: float):
        msg = Float64()
        msg.data = float(height)
        self.lift_pub.publish(msg)

    def compute_navigation_cmd(self, target_x: float, target_y: float, target_yaw: float, max_v=0.25, max_w=0.8):
        dx_world = target_x - self.current_x
        dy_world = target_y - self.current_y
        dist = math.hypot(dx_world, dy_world)

        dyaw = normalize_angle(target_yaw - self.current_yaw)

        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)

        dx_body = dx_world * cos_y + dy_world * sin_y
        dy_body = -dx_world * sin_y + dy_world * cos_y

        kp_pos = 1.2
        kp_yaw = 1.5

        vx = max(min(kp_pos * dx_body, max_v), -max_v)
        vy = max(min(kp_pos * dy_body, max_v), -max_v)
        wz = max(min(kp_yaw * dyaw, max_w), -max_w)

        return vx, vy, wz, dist, abs(dyaw)

    def transition(self, next_state: str):
        self.get_logger().info(f"FSM State Transition: {self.state} -> {next_state}")
        self.state = next_state
        self.state_start_time = time.time()

    def control_loop(self):
        if self.current_x is None:
            return

        elapsed = time.time() - self.state_start_time

        # STATE 0: WAIT_ODOM
        if self.state == self.STATE_WAIT_ODOM:
            self.set_lift(LIFT_HEIGHT_TRANSIT)
            self.publish_twist(0.0, 0.0, 0.0)
            if elapsed > 1.0:
                self.get_logger().info("Navigating to staging position in front of rack...")
                self.transition(self.STATE_NAV_STAGING)

        # STATE 1: NAV_STAGING
        elif self.state == self.STATE_NAV_STAGING:
            self.set_lift(LIFT_HEIGHT_TRANSIT)
            vx, vy, wz, dist, dyaw = self.compute_navigation_cmd(
                self.staging_x, self.staging_y, self.target_yaw, max_v=0.20, max_w=0.6
            )
            self.publish_twist(vx, vy, wz)

            if dist < 0.02 and dyaw < 0.03:
                self.publish_twist(0.0, 0.0, 0.0)
                self.get_logger().info(f"Reached Staging Pose: X={self.current_x:.3f}m, Y={self.current_y:.3f}m")
                self.transition(self.STATE_PRE_LIFT_ALIGN)

        # STATE 2: PRE_LIFT_ALIGN
        elif self.state == self.STATE_PRE_LIFT_ALIGN:
            self.publish_twist(0.0, 0.0, 0.0)
            self.set_lift(self.lift_insert_height)
            if elapsed > 1.5:
                self.get_logger().info(f"Fork aligned at {self.lift_insert_height*100:.2f}cm. Inserting fork...")
                self.transition(self.STATE_FORK_INSERT)

        # STATE 3: FORK_INSERT
        elif self.state == self.STATE_FORK_INSERT:
            self.set_lift(self.lift_insert_height)
            vx, vy, wz, dist, dyaw = self.compute_navigation_cmd(
                self.insert_x, self.insert_y, self.target_yaw, max_v=0.08, max_w=0.4
            )
            vy = max(min(vy, 0.02), -0.02)
            self.publish_twist(vx, vy, wz)

            reached = (self.current_x <= self.insert_x + 0.015) if "left" in self.rack_name else (self.current_y >= self.insert_y - 0.015)
            if reached or elapsed > 6.0:
                self.publish_twist(0.0, 0.0, 0.0)
                self.get_logger().info(f"Fork inserted. Lifting pallet to {self.lift_carry_height*100:.2f}cm...")
                self.transition(self.STATE_LIFT_PALLET)

        # STATE 4: LIFT_PALLET
        elif self.state == self.STATE_LIFT_PALLET:
            self.publish_twist(0.0, 0.0, 0.0)
            self.set_lift(self.lift_carry_height)
            if elapsed > 2.0:
                self.get_logger().info("Pallet lifted! Retracting fork from rack...")
                self.transition(self.STATE_FORK_RETRACT)

        # STATE 5: FORK_RETRACT
        elif self.state == self.STATE_FORK_RETRACT:
            self.set_lift(self.lift_carry_height)
            vx, vy, wz, dist, dyaw = self.compute_navigation_cmd(
                self.staging_x, self.staging_y, self.target_yaw, max_v=0.10, max_w=0.4
            )
            self.publish_twist(vx, vy, wz)

            cleared = (self.current_x >= self.staging_x - 0.02) if "left" in self.rack_name else (self.current_y <= self.staging_y + 0.02)
            if cleared or elapsed > 6.0:
                self.publish_twist(0.0, 0.0, 0.0)
                self.get_logger().info(f"Cleared rack at X={self.current_x:.3f}m, Y={self.current_y:.3f}m. Heading to delivery...")
                self.transition(self.STATE_NAV_DELIVERY)

        # STATE 6: NAV_DELIVERY (to Drop-off Zone or Home Base)
        elif self.state == self.STATE_NAV_DELIVERY:
            self.set_lift(self.lift_carry_height)
            if self.dropoff_target:
                tgt_x = self.dropoff_target.approach_pose.x
                tgt_y = self.dropoff_target.approach_pose.y
                tgt_yaw = self.dropoff_target.approach_pose.yaw
            else:
                tgt_x, tgt_y, tgt_yaw = self.home_x, self.home_y, self.home_yaw

            vx, vy, wz, dist, dyaw = self.compute_navigation_cmd(tgt_x, tgt_y, tgt_yaw, max_v=0.25, max_w=0.6)
            self.publish_twist(vx, vy, wz)

            if dist < 0.03 and dyaw < 0.04:
                self.publish_twist(0.0, 0.0, 0.0)
                self.get_logger().info(f"Arrived at Destination: X={self.current_x:.3f}m, Y={self.current_y:.3f}m. Lowering Pallet...")
                self.transition(self.STATE_LOWER_PALLET)

        # STATE 7: LOWER_PALLET
        elif self.state == self.STATE_LOWER_PALLET:
            self.publish_twist(0.0, 0.0, 0.0)
            self.set_lift(0.0)
            if elapsed > 2.0:
                self.get_logger().info("Pallet successfully placed!")
                self.transition(self.STATE_COMPLETED)

        # STATE 8: COMPLETED
        elif self.state == self.STATE_COMPLETED:
            self.publish_twist(0.0, 0.0, 0.0)
            self.set_lift(0.0)
            self.get_logger().info("================ MISSION ACCOMPLISHED ================")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousPalletMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_twist(0.0, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
