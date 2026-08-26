#!/usr/bin/env python3
"""
Autonomous Mission Node for Robot0 AMR:
Performs fully automated Pick-and-Place Pallet Transport:
Home -> Approach Rack -> Insert Forks -> Lift Pallet -> Reverse ->
Navigate to Drop-off Station -> Insert -> Lower Pallet -> Reverse -> Return Home.
"""

import math
import time
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from robot0_navigation.arena_coordinates import (
    Pose2D,
    HOME_BASE,
    PalletSlot,
    DropOffStation,
    PALLET_SLOTS,
    DROPOFF_STATIONS,
    get_slot_by_type,
    get_slot_by_shelf_and_side,
    get_dropoff_station,
    get_default_dropoff_for_item,
)


class MissionState(Enum):
    INIT = auto()
    NAV_TO_RACK_APPROACH = auto()
    ADJUST_LIFT_APPROACH = auto()
    INSERT_FORKS = auto()
    LIFT_PALLET = auto()
    RETRACT_FROM_RACK = auto()
    NAV_TO_DROPOFF_APPROACH = auto()
    INSERT_DROPOFF = auto()
    LOWER_PALLET = auto()
    RETRACT_FROM_DROPOFF = auto()
    RETURN_HOME = auto()
    COMPLETE = auto()


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AutonomousMissionNode(Node):
    def __init__(self):
        super().__init__('autonomous_mission')

        # Parameters
        self.declare_parameter('pallet_type', 'cpu')
        self.declare_parameter('shelf_level', 1)
        self.declare_parameter('slot_side', 'left')
        self.declare_parameter('dropoff', '')

        pallet_type_param = self.get_parameter('pallet_type').get_parameter_value().string_value
        shelf_param = self.get_parameter('shelf_level').get_parameter_value().integer_value
        slot_param = self.get_parameter('slot_side').get_parameter_value().string_value
        dropoff_param = self.get_parameter('dropoff').get_parameter_value().string_value

        # Select target pallet slot
        self.target_slot: Optional[PalletSlot] = get_slot_by_type(pallet_type_param)
        if self.target_slot is None:
            self.target_slot = get_slot_by_shelf_and_side(shelf_param, slot_param)
        if self.target_slot is None:
            self.target_slot = PALLET_SLOTS["cpu_bottom_left"]

        # Select target dropoff station
        if dropoff_param:
            self.target_dropoff: Optional[DropOffStation] = get_dropoff_station(dropoff_param)
        else:
            self.target_dropoff = get_default_dropoff_for_item(self.target_slot.item_type)

        if self.target_dropoff is None:
            self.target_dropoff = DROPOFF_STATIONS["blue"]

        self.get_logger().info(
            f"=== Mission Configured ==="
            f"\n  Target Pallet: {self.target_slot.name} (Type: {self.target_slot.item_type}, Shelf: {self.target_slot.shelf_level})"
            f"\n  Target Drop-off: {self.target_dropoff.name} (Color: {self.target_dropoff.color})"
        )

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_cmd_pub = self.create_publisher(Float64, '/lift_joint_cmd', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # State tracking
        self.current_pose: Optional[Pose2D] = None
        self.state: MissionState = MissionState.INIT
        self.state_start_time: float = 0.0
        self.target_lift_height: float = 0.0

        # Motion Control Gains
        self.k_lin = 1.2
        self.k_ang = 2.0
        self.max_lin_vel = 0.4
        self.max_ang_vel = 1.0
        self.pos_tolerance = 0.03 # 3cm
        self.yaw_tolerance = 0.05 # ~2.8 deg

        # Main Control Loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_pose = Pose2D(x=x, y=y, yaw=yaw)

    def set_lift(self, height: float):
        self.target_lift_height = height
        msg = Float64()
        msg.data = float(height)
        self.lift_cmd_pub.publish(msg)

    def drive_to_pose(self, target: Pose2D, max_speed: Optional[float] = None) -> bool:
        """Holonomic/Planar Drive to Target Pose2D. Returns True when arrived within tolerance."""
        if self.current_pose is None:
            return False

        max_v = max_speed if max_speed is not None else self.max_lin_vel
        dx = target.x - self.current_pose.x
        dy = target.y - self.current_pose.y
        dist = math.hypot(dx, dy)
        d_yaw = normalize_angle(target.yaw - self.current_pose.yaw)

        if dist < self.pos_tolerance and abs(d_yaw) < self.yaw_tolerance:
            self.stop_robot()
            return True

        # Transform world error into robot body frame
        c = math.cos(self.current_pose.yaw)
        s = math.sin(self.current_pose.yaw)
        body_dx = c * dx + s * dy
        body_dy = -s * dx + c * dy

        cmd = Twist()
        # Proportional velocity control in body frame
        cmd.linear.x = max(min(self.k_lin * body_dx, max_v), -max_v)
        cmd.linear.y = max(min(self.k_lin * body_dy, max_v), -max_v)
        cmd.angular.z = max(min(self.k_ang * d_yaw, self.max_ang_vel), -self.max_ang_vel)

        self.cmd_vel_pub.publish(cmd)
        return False

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def transition_to(self, new_state: MissionState):
        self.get_logger().info(f"Transitioning: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_start_time = time.time()
        self.stop_robot()

    def control_loop(self):
        if self.current_pose is None:
            return

        now = time.time()
        dt_state = now - self.state_start_time

        if self.state == MissionState.INIT:
            self.set_lift(0.0)
            self.transition_to(MissionState.NAV_TO_RACK_APPROACH)

        elif self.state == MissionState.NAV_TO_RACK_APPROACH:
            # 1. Drive to approach pose before the rack bay
            if self.drive_to_pose(self.target_slot.approach_pose):
                self.transition_to(MissionState.ADJUST_LIFT_APPROACH)

        elif self.state == MissionState.ADJUST_LIFT_APPROACH:
            # 2. Adjust lift to approach height
            self.set_lift(self.target_slot.lift_height_approach)
            if dt_state > 1.5:
                self.transition_to(MissionState.INSERT_FORKS)

        elif self.state == MissionState.INSERT_FORKS:
            # 3. Drive forward slowly to insert forks inside pallet runners
            if self.drive_to_pose(self.target_slot.insert_pose, max_speed=0.15):
                self.transition_to(MissionState.LIFT_PALLET)

        elif self.state == MissionState.LIFT_PALLET:
            # 4. Raise lift to carry height
            self.set_lift(self.target_slot.lift_height_carry)
            if dt_state > 2.0:
                self.transition_to(MissionState.RETRACT_FROM_RACK)

        elif self.state == MissionState.RETRACT_FROM_RACK:
            # 5. Reverse back to approach pose
            if self.drive_to_pose(self.target_slot.approach_pose, max_speed=0.15):
                self.transition_to(MissionState.NAV_TO_DROPOFF_APPROACH)

        elif self.state == MissionState.NAV_TO_DROPOFF_APPROACH:
            # 6. Navigate across arena to target dropoff approach pose
            if self.drive_to_pose(self.target_dropoff.approach_pose):
                self.transition_to(MissionState.INSERT_DROPOFF)

        elif self.state == MissionState.INSERT_DROPOFF:
            # 7. Drive slowly into dropoff station
            if self.drive_to_pose(self.target_dropoff.insert_pose, max_speed=0.15):
                self.transition_to(MissionState.LOWER_PALLET)

        elif self.state == MissionState.LOWER_PALLET:
            # 8. Lower pallet onto the dropoff rails
            self.set_lift(self.target_dropoff.lift_height_place)
            if dt_state > 2.0:
                self.transition_to(MissionState.RETRACT_FROM_DROPOFF)

        elif self.state == MissionState.RETRACT_FROM_DROPOFF:
            # 9. Reverse out of dropoff station
            if self.drive_to_pose(self.target_dropoff.approach_pose, max_speed=0.15):
                self.transition_to(MissionState.RETURN_HOME)

        elif self.state == MissionState.RETURN_HOME:
            # 10. Return to Home Dock
            if self.drive_to_pose(HOME_BASE):
                self.transition_to(MissionState.COMPLETE)

        elif self.state == MissionState.COMPLETE:
            self.stop_robot()
            self.get_logger().info("🎉 MISSION COMPLETE! Pallet successfully delivered.")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
