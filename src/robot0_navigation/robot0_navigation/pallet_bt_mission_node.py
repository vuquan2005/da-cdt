#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Behavior Tree Pallet Mission Node for Robot0.
Coordinates autonomous pallet retrieval from warehouse racks to designated drop-off locations
using a modular and expandable Behavior Tree architecture.
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
    Builds the Complete Behavior Tree for Pallet Pick-and-Place:
    1. Initialize Blackboard & Wait for Odometry
    2. Simulated Line Navigation to Staging Pose in front of Rack
    3. Align Fork Height -> Insert Fork -> Raise Lift -> Retract Fork
    4. Simulated Line Navigation with Pallet to Drop-off Zone
    5. Lower Pallet to Ground -> Backoff from Pallet
    6. Simulated Line Navigation to Return to Home Base
    """
    root = Sequence('Pallet_Mission_Master_Tree', blackboard=blackboard)

    # ---------------- 1. INITIALIZATION ----------------
    init_seq = Sequence('1_Initialization')
    init_seq.add_child(InitializeMissionAction('Init_Mission_Coordinates'))
    init_seq.add_child(WaitForOdometryCondition('Wait_For_Odometry'))
    init_seq.add_child(SetLiftHeightAction('Set_Transit_Height', target_height='lift_transit_height', settle_time_sec=1.0))
    root.add_child(init_seq)

    # ---------------- 2. APPROACH RACK (LINE FOLLOWING INTERSECTIONS) ----------------
    approach_seq = Sequence('2_Approach_Rack')
    approach_seq.add_child(LogMessageAction('Log_Nav_Staging', 'Đang dò line qua các điểm giao tới vị trí trước kệ...'))
    approach_seq.add_child(NavigateThroughWaypointsAction('Line_Nav_To_Staging', waypoints_spec='approach_route', pos_tolerance=0.025, yaw_tolerance=0.035, max_v=0.22))
    approach_seq.add_child(LogMessageAction('Log_Staging_Reached', 'Đã đến vị trí chuẩn bị trước kệ!'))
    root.add_child(approach_seq)

    # ---------------- 3. PICK PALLET FROM RACK ----------------
    pick_seq = Sequence('3_Pick_Pallet')
    pick_seq.add_child(LogMessageAction('Log_Align_Height', 'Căn chỉnh độ cao càng nâng vào khe pallet...'))
    pick_seq.add_child(SetLiftHeightAction('Align_Fork_To_Slot', target_height='lift_insert_height', settle_time_sec=2.5))
    pick_seq.add_child(LogMessageAction('Log_Insert_Fork', 'Tiến càng vào sâu bên dưới pallet...'))
    pick_seq.add_child(NavigateToPoseAction('Insert_Fork', target_pose='insert_pose', pos_tolerance=0.015, yaw_tolerance=0.03, max_v=0.07, timeout_sec=8.0, is_insert_mode=True))
    pick_seq.add_child(WaitAction('Settle_Before_Lift', 0.5))
    pick_seq.add_child(LogMessageAction('Log_Raise_Lift', 'Nhấc pallet lên khỏi mặt kệ...'))
    pick_seq.add_child(SetLiftHeightAction('Raise_Pallet_To_Carry', target_height='lift_carry_height', settle_time_sec=2.5))
    pick_seq.add_child(LogMessageAction('Log_Retract_Fork', 'Lùi xe rút càng mang pallet ra khỏi kệ...'))
    pick_seq.add_child(NavigateToPoseAction('Retract_From_Rack', target_pose='staging_pose', pos_tolerance=0.020, yaw_tolerance=0.03, max_v=0.10))
    pick_seq.add_child(LogMessageAction('Log_Pick_Success', 'Đã lấy pallet ra khỏi kệ an toàn!'))
    root.add_child(pick_seq)

    # ---------------- 4. DELIVER TO DROP-OFF ZONE (LINE FOLLOWING INTERSECTIONS) ----------------
    deliver_seq = Sequence('4_Deliver_To_Destination')
    deliver_seq.add_child(LogMessageAction('Log_Nav_Delivery', 'Vận chuyển pallet qua mạng lưới line tới vị trí giao hàng...'))
    deliver_seq.add_child(NavigateThroughWaypointsAction('Line_Nav_To_Dropoff', waypoints_spec='delivery_route', pos_tolerance=0.030, yaw_tolerance=0.040, max_v=0.25))
    deliver_seq.add_child(LogMessageAction('Log_Dropoff_Arrived', 'Đã đến khu vực giao hàng!'))
    root.add_child(deliver_seq)

    # ---------------- 5. PLACE PALLET ----------------
    place_seq = Sequence('5_Place_Pallet')
    place_seq.add_child(LogMessageAction('Log_Lower_Pallet', 'Hạ càng đặt pallet xuống vị trí chỉ định...'))
    place_seq.add_child(SetLiftHeightAction('Lower_Pallet_To_Ground', target_height='lift_dropoff_height', settle_time_sec=2.0))
    place_seq.add_child(WaitAction('Settle_After_Drop', 0.5))
    place_seq.add_child(LogMessageAction('Log_Backoff', 'Lùi xe ra khỏi pallet đã hạ...'))
    place_seq.add_child(LinearDriveAction('Backoff_From_Pallet', distance_meters=-0.25, speed=0.10))
    place_seq.add_child(LogMessageAction('Log_Pallet_Placed', 'Pallet đã được đặt thành công!'))
    root.add_child(place_seq)

    # ---------------- 6. RETURN HOME BASE (LINE FOLLOWING INTERSECTIONS) ----------------
    home_seq = Sequence('6_Return_Home')
    home_seq.add_child(LogMessageAction('Log_Return_Home', 'Dò line di chuyển về vị trí xuất phát ban đầu...'))
    home_seq.add_child(SetLiftHeightAction('Lift_Safe_Transit', target_height='lift_transit_height', settle_time_sec=1.0))
    home_seq.add_child(NavigateThroughWaypointsAction('Line_Nav_To_Home', waypoints_spec='return_home_route', pos_tolerance=0.030, yaw_tolerance=0.040, max_v=0.25))
    home_seq.add_child(LogMessageAction('Log_Mission_Success', '================ MISSION ACCOMPLISHED ================'))
    root.add_child(home_seq)

    return BehaviorTree(root, blackboard)

    return BehaviorTree(root, blackboard)


class PalletBTMissionNode(Node):
    """Main ROS 2 Node executing the Pallet Mission Behavior Tree."""
    def __init__(self):
        super().__init__('pallet_bt_mission_node')

        # Declare ROS 2 Parameters
        self.declare_parameter('target_rack', 'rack_1')
        self.declare_parameter('shelf_level', 1)
        self.declare_parameter('target_slot', 'left')
        self.declare_parameter('pallet_type', '')      # e.g. 'aluminum', 'cpu', 'qr', 'chip'
        self.declare_parameter('dropoff_zone', '')     # e.g. 'dropoff_1', 'dropoff_2', 'dropoff_3', 'dropoff_4', 'home'
        self.declare_parameter('tick_rate_hz', 20.0)
        self.declare_parameter('print_tree_interval_sec', 3.0)

        # Mecanum Robot Geometry
        self.wheel_radius = 0.0487  # m
        self.lx = 0.1000            # Half wheelbase (m)
        self.ly = 0.1539            # Half track width (m)
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
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Continuous state
        self.target_lift_pos = 0.015

        # Behavior Tree & Blackboard Setup
        self.blackboard = Blackboard()
        self.blackboard.set('ros_node', self)
        self.blackboard.set('param_target_rack', self.get_parameter('target_rack').value)
        self.blackboard.set('param_shelf_level', self.get_parameter('shelf_level').value)
        self.blackboard.set('param_target_slot', self.get_parameter('target_slot').value)
        self.blackboard.set('param_pallet_type', self.get_parameter('pallet_type').value)
        self.blackboard.set('param_dropoff_zone', self.get_parameter('dropoff_zone').value)

        self.tree = build_pallet_mission_tree(self.blackboard)

        # Timers
        tick_period = 1.0 / float(self.get_parameter('tick_rate_hz').value)
        self.tick_timer = self.create_timer(tick_period, self.tree_tick_loop)

        self.print_tree_interval = float(self.get_parameter('print_tree_interval_sec').value)
        self.last_print_time = time.time()
        self.mission_completed = False

        self.get_logger().info('Pallet Mission Behavior Tree Node Started successfully.')

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
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(twist)

        # Synchronize wheel velocities
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
        self.target_lift_pos = float(height)
        msg = Float64()
        msg.data = float(height)
        self.lift_pub.publish(msg)

    def tree_tick_loop(self):
        if self.mission_completed:
            return

        # Continuously publish active lift position target to Gazebo
        lift_msg = Float64()
        lift_msg.data = float(self.target_lift_pos)
        self.lift_pub.publish(lift_msg)

        status = self.tree.tick()

        # Print tree visualization periodically
        now = time.time()
        if now - self.last_print_time >= self.print_tree_interval:
            self.last_print_time = now
            print('\n' + '=' * 60)
            print(f'Behavior Tree Snapshot (Tick #{self.tree.tick_count}):')
            print(self.tree.render_ascii_tree(use_color=True))
            print('=' * 60 + '\n')

        if status == NodeStatus.SUCCESS:
            self.mission_completed = True
            self.publish_twist(0.0, 0.0, 0.0)
            self.get_logger().info('Pallet Mission Completed with SUCCESS!')
            print('\n' + '=' * 60)
            print('FINAL BEHAVIOR TREE STATUS:')
            print(self.tree.render_ascii_tree(use_color=True))
            print('=' * 60 + '\n')
            self.tick_timer.cancel()
        elif status == NodeStatus.FAILURE:
            self.mission_completed = True
            self.publish_twist(0.0, 0.0, 0.0)
            self.get_logger().error('Pallet Mission FAILED!')
            print('\n' + '=' * 60)
            print('FINAL BEHAVIOR TREE STATUS (FAILURE):')
            print(self.tree.render_ascii_tree(use_color=True))
            print('=' * 60 + '\n')
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
