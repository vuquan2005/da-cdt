#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Behavior Tree Pallet Mission Node for Robot0 (Skeleton / Architecture).
Coordinates autonomous pallet retrieval from warehouse racks to designated drop-off locations
using a modular Behavior Tree architecture.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

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
    NavigateToPoseAction,
    NavigateThroughWaypointsAction,
    LinearDriveAction,
)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def build_pallet_mission_tree(blackboard: Blackboard) -> BehaviorTree:
    """
    Builds the Pallet Pick-and-Place Behavior Tree Skeleton:
    1. Initialization: Prepare coordinates, wait for odom, transit height.
    2. Approach Rack: Navigate to staging pose before rack.
    3. Pick Pallet: Align height, insert fork, lift pallet, retract.
    4. Deliver: Navigate to drop-off destination.
    5. Place Pallet: Lower pallet, back off.
    6. Return Home: Navigate back to home spawn base.
    """
    root = Sequence('Pallet_Mission_Master_Tree', blackboard=blackboard)

    # 1. Initialization
    init_seq = Sequence('1_Initialization')
    init_seq.add_child(InitializeMissionAction('Init_Mission_Coordinates'))
    init_seq.add_child(WaitForOdometryCondition('Wait_For_Odometry'))
    init_seq.add_child(SetLiftHeightAction('Set_Transit_Height', target_height='lift_transit_height', settle_time_sec=1.0))
    root.add_child(init_seq)

    # 2. Approach Rack
    approach_seq = Sequence('2_Approach_Rack')
    approach_seq.add_child(LogMessageAction('Log_Nav_Rack_Line', 'Tiếp cận vị trí trước kệ trên trục chính...'))
    approach_seq.add_child(NavigateThroughWaypointsAction('Line_Nav_To_Rack', waypoints_spec='approach_route', pos_tolerance=0.015, transit_radius=0.06))
    approach_seq.add_child(LogMessageAction('Log_Rack_Reached', 'Đã đến vị trí trước kệ trên trục chính!'))
    root.add_child(approach_seq)

    # 3. Pick Pallet from Rack (Decoupled Orthogonal Pick)
    pick_seq = Sequence('3_Pick_Pallet')
    pick_seq.add_child(LogMessageAction('Log_Shift_Slot', 'Dạt ngang 60mm vào đúng tim khay pallet...'))
    pick_seq.add_child(NavigateToPoseAction('Shift_To_Pallet_Slot', target_pose='staging_pose', pos_tolerance=0.008))
    pick_seq.add_child(LogMessageAction('Log_Align_Height', 'Căn chỉnh độ cao càng nâng...'))
    pick_seq.add_child(SetLiftHeightAction('Align_Fork_To_Slot', target_height='lift_insert_height', settle_time_sec=0.8))
    pick_seq.add_child(LogMessageAction('Log_Insert_Fork', 'Tiến thẳng 14.5cm xỏ càng vào pallet...'))
    pick_seq.add_child(LinearDriveAction('Insert_Fork_Straight', distance_meters=0.145, axis='x', speed=0.06, tolerance=0.006))
    pick_seq.add_child(WaitAction('Settle_Before_Lift', 0.5))
    pick_seq.add_child(LogMessageAction('Log_Raise_Lift', 'Nhấc pallet lên khỏi mặt kệ...'))
    pick_seq.add_child(SetLiftHeightAction('Raise_Pallet_To_Carry', target_height='lift_carry_height', settle_time_sec=0.8))
    pick_seq.add_child(LogMessageAction('Log_Retract_Fork', 'Lùi thẳng 14.5cm mang pallet ra khỏi kệ...'))
    pick_seq.add_child(LinearDriveAction('Retract_From_Rack_Straight', distance_meters=-0.145, axis='x', speed=0.06, tolerance=0.006))
    pick_seq.add_child(LogMessageAction('Log_Shift_Back', 'Dạt ngang 60mm trở lại tim đường chính...'))
    pick_seq.add_child(NavigateToPoseAction('Shift_Back_To_Main_Line', target_pose='rack_approach_pose', pos_tolerance=0.015))
    pick_seq.add_child(LogMessageAction('Log_Pick_Success', 'Đã lấy pallet ra khỏi kệ thành công!'))
    root.add_child(pick_seq)

    # 4. Deliver to Destination
    deliver_seq = Sequence('4_Deliver_To_Destination')
    deliver_seq.add_child(LogMessageAction('Log_Nav_Delivery', 'Vận chuyển pallet tới vị trí giao hàng...'))
    deliver_seq.add_child(NavigateThroughWaypointsAction('Line_Nav_To_Dropoff', waypoints_spec='delivery_route'))
    deliver_seq.add_child(LogMessageAction('Log_Dropoff_Arrived', 'Đã đến khu vực giao hàng!'))
    root.add_child(deliver_seq)

    # 5. Place Pallet
    place_seq = Sequence('5_Place_Pallet')
    place_seq.add_child(LogMessageAction('Log_Lower_Pallet', 'Hạ càng đặt pallet...'))
    place_seq.add_child(SetLiftHeightAction('Lower_Pallet_To_Ground', target_height='lift_dropoff_height', settle_time_sec=0.8))
    place_seq.add_child(WaitAction('Settle_After_Drop', 0.5))
    place_seq.add_child(LogMessageAction('Log_Backoff', 'Lùi xe ra khỏi pallet...'))
    place_seq.add_child(LinearDriveAction('Backoff_From_Pallet', distance_meters=-0.25, speed=0.10))
    place_seq.add_child(LogMessageAction('Log_Pallet_Placed', 'Pallet đã được đặt thành công!'))
    root.add_child(place_seq)

    # 6. Return Home
    home_seq = Sequence('6_Return_Home')
    home_seq.add_child(LogMessageAction('Log_Return_Home', 'Di chuyển về vị trí xuất phát...'))
    home_seq.add_child(SetLiftHeightAction('Lift_Safe_Transit', target_height='lift_transit_height', settle_time_sec=1.0))
    home_seq.add_child(NavigateThroughWaypointsAction('Line_Nav_To_Home', waypoints_spec='return_home_route'))
    home_seq.add_child(LogMessageAction('Log_Mission_Success', '================ MISSION COMPLETED ================'))
    root.add_child(home_seq)

    return BehaviorTree(root, blackboard)


class PalletBTMissionNode(Node):
    """Main ROS 2 Node executing the Pallet Mission Behavior Tree Skeleton."""
    def __init__(self):
        super().__init__('pallet_bt_mission_node')

        # Parameters
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

        # State
        self.target_lift_pos = 0.015
        self.mission_completed = False

        # Behavior Tree & Blackboard Setup
        self.blackboard = Blackboard()
        self.blackboard.set('ros_node', self)
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

        self.get_logger().info('Pallet Mission Behavior Tree Node Started.')

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
