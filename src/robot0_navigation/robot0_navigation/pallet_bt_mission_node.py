#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Behavior Tree Pallet Mission Node for Robot0 (Skeleton / Architecture).
Coordinates autonomous pallet retrieval from warehouse racks to designated drop-off locations
using a modular Behavior Tree architecture.
"""

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String

from robot0_navigation.behavior_tree import (
    BehaviorTree,
    Blackboard,
    NodeStatus,
    Sequence,
    Selector,
    Parallel,
    Inverter,
    RetryNode,
)
from robot0_navigation.behaviors import (
    LogMessageAction,
    WaitAction,
    WaitForOdometryCondition,
    InitializeMissionAction,
    SetLiftHeightAction,
    ScanRackPalletsWithYoloAction,
    NavigateToPoseAction,
    NavigateThroughWaypointsAction,
    LinearDriveAction,
)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _build_pick_and_deliver_subsequence(prefix: str, rack_name: str) -> Sequence:
    """
    Subsequence: Gắp pallet từ kệ -> Vận chuyển đến Dropoff Zone -> Đặt pallet -> Trở về Home Base.
    """
    seq = Sequence(f'{prefix}_Pick_Deliver_Return')

    # 1. Pick Pallet from Rack
    seq.add_child(LogMessageAction(f'Log_{prefix}_Shift', f'Dạt ngang vào đúng tim khay pallet tại {rack_name}...'))
    seq.add_child(NavigateToPoseAction(f'{prefix}_Shift_To_Slot', target_pose='staging_pose', pos_tolerance=0.004))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Align_Lift', 'Căn chỉnh độ cao càng nâng...'))
    seq.add_child(SetLiftHeightAction(f'{prefix}_Align_Fork', target_height='lift_insert_height', settle_time_sec=0.8))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Insert', 'Tiến thẳng 14.5cm xỏ càng vào pallet...'))
    seq.add_child(LinearDriveAction(f'{prefix}_Insert_Fork', distance_meters=0.145, axis='x', speed=0.06, tolerance=0.006))
    seq.add_child(WaitAction(f'{prefix}_Settle_Lift', 0.5))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Raise', 'Nhấc pallet lên khỏi mặt kệ...'))
    seq.add_child(SetLiftHeightAction(f'{prefix}_Raise_Pallet', target_height='lift_carry_height', settle_time_sec=0.8))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Retract', 'Lùi thẳng 14.5cm mang pallet ra khỏi kệ...'))
    seq.add_child(LinearDriveAction(f'{prefix}_Retract_Fork', distance_meters=-0.145, axis='x', speed=0.06, tolerance=0.006))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Shift_Back', 'Dạt ngang trở lại tim đường chính...'))
    seq.add_child(NavigateToPoseAction(f'{prefix}_Shift_Back_To_Main', target_pose='rack_approach_pose', pos_tolerance=0.015))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Pick_Done', f'Đã lấy pallet ra khỏi {rack_name} thành công!'))

    # 2. Deliver to Destination
    seq.add_child(LogMessageAction(f'Log_{prefix}_Deliver', 'Vận chuyển pallet tới đúng vị trí giao hàng theo loại hàng...'))
    seq.add_child(NavigateThroughWaypointsAction(f'{prefix}_Nav_Dropoff', waypoints_spec='delivery_route'))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Dropoff_Arrived', 'Đã đến khu vực giao hàng!'))

    # 3. Place Pallet
    seq.add_child(LogMessageAction(f'Log_{prefix}_Lower', 'Hạ càng đặt pallet...'))
    seq.add_child(SetLiftHeightAction(f'{prefix}_Lower_Pallet', target_height='lift_dropoff_height', settle_time_sec=0.8))
    seq.add_child(WaitAction(f'{prefix}_Settle_Drop', 0.5))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Backoff', 'Lùi xe ra khỏi pallet...'))
    seq.add_child(LinearDriveAction(f'{prefix}_Backoff_Pallet', distance_meters=-0.25, speed=0.10))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Placed', 'Pallet đã được đặt thành công!'))

    # 4. Return Home
    seq.add_child(LogMessageAction(f'Log_{prefix}_Return_Home', 'Di chuyển về vị trí xuất phát...'))
    seq.add_child(SetLiftHeightAction(f'{prefix}_Lift_Transit', target_height='lift_transit_height', settle_time_sec=1.0))
    seq.add_child(NavigateThroughWaypointsAction(f'{prefix}_Nav_Home', waypoints_spec='return_home_route'))
    seq.add_child(LogMessageAction(f'Log_{prefix}_Success', f'================ MISSION COMPLETED VIA {rack_name.upper()} ================'))

    return seq


def build_pallet_mission_tree(blackboard: Blackboard) -> BehaviorTree:
    """
    Builds the Dynamic Multi-Rack Pallet Search & Retrieval Behavior Tree:
    - Root: Selector
      ├── Branch 1 (Sequence): Search & Retrieve Execution
      │     ├── 1_Initialization (Init Blackboard, Wait for Odometry, Transit Lift)
      │     └── 2_Search_Racks_Selector (Selector)
      │           ├── 2A_Try_Rack_1 (Nav to Rack 1 -> YOLO Scan Rack 1 -> Pick & Deliver & Return Home)
      │           └── 2B_Try_Rack_2 (Nav to Rack 2 -> YOLO Scan Rack 2 -> Pick & Deliver & Return Home)
      └── Branch 2 (Sequence): Abort Return Home (When pallet not found on BOTH racks)
            ├── Log Warning ("Không tìm thấy pallet ở cả 2 kệ...")
            ├── Set Lift Safe Transit
            └── Navigate Waypoints from Rack 2 back to Home Base
    """
    root = Selector('Master_Pallet_Search_And_Retrieve_Mission', blackboard=blackboard)

    # =========================================================================
    # Branch 1: Search & Retrieve Execution
    # =========================================================================
    search_exec_seq = Sequence('1_Search_And_Retrieve_Flow')

    # Step 1: Initialization
    init_seq = Sequence('1A_Initialization')
    init_seq.add_child(InitializeMissionAction('Init_Mission_Parameters'))
    init_seq.add_child(WaitForOdometryCondition('Wait_For_Odometry'))
    init_seq.add_child(SetLiftHeightAction('Set_Transit_Height', target_height='lift_transit_height', settle_time_sec=1.0))
    search_exec_seq.add_child(init_seq)

    # Step 2: Search Racks Selector (Try Rack 1 first, if not found then Try Rack 2)
    search_racks_sel = Selector('1B_Search_Racks_Selector')

    # --- 2A: Try Rack 1 ---
    try_rack1_seq = Sequence('Try_Rack_1_Flow')
    try_rack1_seq.add_child(LogMessageAction('Log_Nav_Rack1', 'Tiếp cận Kệ 1 để tìm kiếm pallet...'))
    try_rack1_seq.add_child(NavigateThroughWaypointsAction('Nav_To_Rack_1', waypoints_spec='approach_route_rack1', pos_tolerance=0.015, transit_radius=0.06))
    try_rack1_seq.add_child(LogMessageAction('Log_Scan_Rack1', 'Đang quét và nhận diện các pallet tại Kệ 1 bằng YOLO...'))
    try_rack1_seq.add_child(ScanRackPalletsWithYoloAction('Scan_Rack_1', current_rack='rack_1', scan_duration_sec=1.5, timeout_sec=4.0))
    try_rack1_seq.add_child(LogMessageAction('Log_Found_Rack1', '✅ Đã tìm thấy Pallet tại Kệ 1! Bắt đầu gắp hàng...'))
    try_rack1_seq.add_child(_build_pick_and_deliver_subsequence('R1', 'rack_1'))
    search_racks_sel.add_child(try_rack1_seq)

    # --- 2B: Try Rack 2 ---
    try_rack2_seq = Sequence('Try_Rack_2_Flow')
    try_rack2_seq.add_child(LogMessageAction('Log_Nav_Rack2', 'Không thấy ở Kệ 1 -> Di chuyển sang Kệ 2 để tìm kiếm...'))
    try_rack2_seq.add_child(NavigateThroughWaypointsAction('Nav_Rack1_To_Rack2', waypoints_spec='route_rack1_to_rack2', pos_tolerance=0.015, transit_radius=0.06))
    try_rack2_seq.add_child(LogMessageAction('Log_Scan_Rack2', 'Đang quét và nhận diện các pallet tại Kệ 2 bằng YOLO...'))
    try_rack2_seq.add_child(ScanRackPalletsWithYoloAction('Scan_Rack_2', current_rack='rack_2', scan_duration_sec=1.5, timeout_sec=4.0))
    try_rack2_seq.add_child(LogMessageAction('Log_Found_Rack2', '✅ Đã tìm thấy Pallet tại Kệ 2! Bắt đầu gắp hàng...'))
    try_rack2_seq.add_child(_build_pick_and_deliver_subsequence('R2', 'rack_2'))
    search_racks_sel.add_child(try_rack2_seq)

    search_exec_seq.add_child(search_racks_sel)
    root.add_child(search_exec_seq)

    # =========================================================================
    # Branch 2: Fallback Abort (If pallet was NOT found at both Rack 1 and Rack 2)
    # =========================================================================
    abort_seq = Sequence('2_Abort_Return_Home_When_Not_Found')
    abort_seq.add_child(LogMessageAction('Log_Not_Found_Both', '⚠️ KHÔNG TÌM THẤY PALLET Ở CẢ 2 KỆ! Rút lui an toàn về Home Base...'))
    abort_seq.add_child(SetLiftHeightAction('Lift_Transit_Abort', target_height='lift_transit_height', settle_time_sec=0.5))
    abort_seq.add_child(NavigateThroughWaypointsAction('Nav_Home_From_Rack2', waypoints_spec='route_rack2_to_home', pos_tolerance=0.015, transit_radius=0.06))
    abort_seq.add_child(LogMessageAction('Log_Mission_Aborted', '================ PALLET NOT FOUND - RETURNED HOME SAFELY ================'))
    root.add_child(abort_seq)

    return BehaviorTree(root, blackboard)


class PalletBTMissionNode(Node):
    """Main ROS 2 Node executing the Pallet Mission Behavior Tree Skeleton."""
    def __init__(self):
        super().__init__('pallet_bt_mission_node')

        # Parameters
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('target_rack', 'rack_1')
        self.declare_parameter('shelf_level', 1)
        self.declare_parameter('target_slot', 'left')
        self.declare_parameter('pallet_type', '')
        self.declare_parameter('dropoff_zone', '')
        self.declare_parameter('tick_rate_hz', 20.0)
        self.declare_parameter('print_tree_interval_sec', 3.0)

        # Standard ROS 2 Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/lift_joint_cmd', 10)

        # Standard ROS 2 Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.yolo_sub = self.create_subscription(String, '/yolo/detections_json', self.yolo_callback, 10)

        # State
        self.target_lift_pos = 0.015
        self.mission_completed = False

        # Behavior Tree & Blackboard Setup
        self.blackboard = Blackboard()
        self.blackboard.set('ros_node', self)
        self.blackboard.set('param_use_yolo', bool(self.get_parameter('use_yolo').value))
        self.blackboard.set('param_target_rack', self.get_parameter('target_rack').value)
        self.blackboard.set('param_shelf_level', self.get_parameter('shelf_level').value)
        self.blackboard.set('param_target_slot', self.get_parameter('target_slot').value)
        self.blackboard.set('param_pallet_type', self.get_parameter('pallet_type').value)
        self.blackboard.set('param_dropoff_zone', self.get_parameter('dropoff_zone').value)

        self.tree = build_pallet_mission_tree(self.blackboard)

        # Periodic Tick Timer
        tick_period = 1.0 / float(self.get_parameter('tick_rate_hz').value)
        self.tick_timer = self.create_timer(tick_period, self.tree_tick_loop)

        self.print_tree_interval = float(self.get_parameter('print_tree_interval_sec').value)
        self.last_print_time = time.time()

        self.get_logger().info('Pallet Mission Behavior Tree Node Started (YOLO Integration Active).')

    def yolo_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.blackboard.set('latest_yolo_detections', data)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse YOLO detections JSON: {e}")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quat_to_yaw(ori.x, ori.y, ori.z, ori.w)

        self.blackboard.set('current_x', pos.x)
        self.blackboard.set('current_y', pos.y)
        self.blackboard.set('current_yaw', yaw)

    def joint_state_callback(self, msg: JointState):
        if 'lift_arm_joint' in msg.name:
            idx = msg.name.index('lift_arm_joint')
            if len(msg.position) > idx:
                self.blackboard.set('actual_lift_pos', float(msg.position[idx]))

    def publish_twist(self, vx: float, vy: float, wz: float):
        """Publishes standard Twist command to /cmd_vel."""
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(twist)

    def set_lift(self, height: float):
        """Commands target lift height to /lift_joint_cmd."""
        self.target_lift_pos = float(height)
        msg = Float64()
        msg.data = float(height)
        self.lift_pub.publish(msg)

    def tree_tick_loop(self):
        if self.mission_completed:
            return

        # Keep commanding lift position
        lift_msg = Float64()
        lift_msg.data = float(self.target_lift_pos)
        self.lift_pub.publish(lift_msg)

        status = self.tree.tick()

        now = time.time()
        if now - self.last_print_time >= self.print_tree_interval:
            self.last_print_time = now
            print('\n' + '=' * 60)
            print(f'Behavior Tree Snapshot (Tick #{self.tree.tick_count}):')
            print(self.tree.render_ascii_tree(use_color=True))
            print('=' * 60 + '\n')

        if status in (NodeStatus.SUCCESS, NodeStatus.FAILURE):
            self.mission_completed = True
            self.publish_twist(0.0, 0.0, 0.0)
            result_str = 'SUCCESS' if status == NodeStatus.SUCCESS else 'FAILURE'
            self.get_logger().info(f'Pallet Mission Finished with status: {result_str}')
            self.tick_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PalletBTMissionNode()
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
