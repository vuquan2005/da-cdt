#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Autonomous Pallet Retrieval Mission Node for Robot0.

Physics & Geometry Reference:
- Base footprint to base_link: Z = +0.060 m
- base_link to lift_arm_joint: Z = +0.0435 m
- lift_arm_joint to fork blade center: Z = -0.1115 m
- Absolute Fork Blade Center Height above ground: Z_fork(L) = L - 0.008 m

Shelf & Pallet Geometry:
- Tầng 1 (Bottom Shelf surface): Z = 0.0285 m
  Pallet slot opening: Z in [0.0285m, 0.0485m] -> Midpoint Z = 0.0385 m
  => Pre-insert Lift Command L = 0.0385 + 0.008 = 0.0465 m (~4.65 cm)
  => Lift to Carry Command L = 0.0465 + 0.040 = 0.0865 m (~8.65 cm)

- Tầng 2 (Middle Shelf surface): Z = 0.1485 m
  Pallet slot opening: Z in [0.1485m, 0.1685m] -> Midpoint Z = 0.1585 m
  => Pre-insert Lift Command L = 0.1585 + 0.008 = 0.1665 m (~16.65 cm)
  => Lift to Carry Command L = 0.1665 + 0.030 = 0.1965 m (~19.65 cm)
"""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


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
    STATE_NAV_APPROACH = "NAV_APPROACH"
    STATE_PRE_LIFT_ALIGN = "PRE_LIFT_ALIGN"
    STATE_FORK_INSERT = "FORK_INSERT"
    STATE_LIFT_PALLET = "LIFT_PALLET"
    STATE_FORK_RETRACT = "FORK_RETRACT"
    STATE_NAV_RETURN = "NAV_RETURN"
    STATE_LOWER_PALLET = "LOWER_PALLET"
    STATE_COMPLETED = "COMPLETED"

    def __init__(self):
        super().__init__('autonomous_pallet_mission')

        # Declare ROS 2 parameters
        self.declare_parameter('target_shelf_level', 1)  # 1 for Bottom shelf, 2 for Middle shelf
        self.declare_parameter('target_slot', 'left')     # 'left' or 'right'

        self.shelf_level = self.get_parameter('target_shelf_level').get_parameter_value().integer_value
        self.slot = self.get_parameter('target_slot').get_parameter_value().string_value.lower()

        # Robot Geometry for wheel sync
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
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.home_x = None
        self.home_y = None
        self.home_yaw = None

        # Mission State
        self.state = self.STATE_WAIT_ODOM
        self.state_start_time = time.time()

        # Rack Coordinates (rack_left_bot)
        # CRITICAL KINEMATIC CORRECTION:
        # lift_arm_joint has an X=+8.27mm offset in base_link (=-8.27mm in base_footprint / =+8.27mm in world Y at yaw=pi).
        # Therefore, robot Y must be shifted by -8.27mm to center the fork blades with the pallet slot!
        pallet_y = 0.5810 if self.slot == 'left' else 0.7010
        self.target_y = pallet_y - 0.00827  # Exact lateral centering!

        # Fork reach is 23.08cm ahead of robot. Pallet center is at X = -1.894m.
        # Deep insertion: X = -1.685m (fork penetrates 2.5cm deeper for rock-solid pallet seating)
        self.staging_x = -1.450
        self.insert_x = -1.685
        self.target_yaw = math.pi  # Facing towards rack (-X)

        # Precise Lift Height Calculation according to Shelf Level (Ground-truth empirical values)
        self.lift_safe_transit = 0.015  # 1.5cm up during navigation to prevent scraping floor
        if self.shelf_level == 2:
            # Tầng 2: Middle shelf (Z = 0.1485m, Delta Z = +0.120m so với Tầng 1)
            self.lift_insert_height = 0.1495  # 14.95 cm (Khớp chuẩn xác tâm khe pallet T2)
            self.lift_carry_height = 0.1850   # 18.50 cm (Nhấc bổng +3.5cm khỏi mặt kệ)
            shelf_desc = "Tầng 2 (Middle Shelf, Z=0.1485m)"
        else:
            # Tầng 1: Bottom shelf (Z = 0.0285m, đo thực nghiệm TF: 0.133m - 0.1035m = 0.0295m)
            self.lift_insert_height = 0.0295  # 2.95 cm (Khớp chuẩn xác tâm khe pallet T1)
            self.lift_carry_height = 0.0700   # 7.00 cm (Nhấc bổng +4.0cm khỏi mặt kệ)
            shelf_desc = "Tầng 1 (Bottom Shelf, Z=0.0285m)"

        # Control Loop Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("=================================================================")
        self.get_logger().info(f"Robot0 Mission Configured for: {shelf_desc}, Ô {self.slot.upper()}")
        self.get_logger().info(f"  Target Y = {self.target_y:.3f}m | Staging X = {self.staging_x:.3f}m | Insert X = {self.insert_x:.3f}m")
        self.get_logger().info(f"  Pre-Insert Lift Height: {self.lift_insert_height*100:.2f} cm (Khớp độ cao khe pallet)")
        self.get_logger().info(f"  Lift to Carry Height  : {self.lift_carry_height*100:.2f} cm (Nhấc bổng khỏi mặt kệ)")
        self.get_logger().info("=================================================================")

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
            self.get_logger().info(f"Spawn Pose Locked: X={self.home_x:.3f}m, Y={self.home_y:.3f}m, Yaw={math.degrees(self.home_yaw):.1f}°")

    def publish_twist(self, vx: float, vy: float, wz: float):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(twist)

        # Synchronized wheels
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
        self.get_logger().info(f"Phase Transition: {self.state} -> {next_state}")
        self.state = next_state
        self.state_start_time = time.time()

    def control_loop(self):
        if self.current_x is None:
            return

        elapsed = time.time() - self.state_start_time

        # -------------------------------------------------------------
        # STATE 0: WAIT_ODOM
        # -------------------------------------------------------------
        if self.state == self.STATE_WAIT_ODOM:
            self.set_lift(self.lift_safe_transit)
            self.publish_twist(0.0, 0.0, 0.0)
            if elapsed > 1.0:
                self.get_logger().info("Step 1: Navigating to staging waypoint in front of rack...")
                self.transition(self.STATE_NAV_APPROACH)

        # -------------------------------------------------------------
        # STATE 1: NAV_APPROACH (Transit to staging position)
        # -------------------------------------------------------------
        elif self.state == self.STATE_NAV_APPROACH:
            self.set_lift(self.lift_safe_transit)
            vx, vy, wz, dist, dyaw = self.compute_navigation_cmd(
                self.staging_x, self.target_y, self.target_yaw, max_v=0.20, max_w=0.6
            )
            self.publish_twist(vx, vy, wz)

            if dist < 0.02 and dyaw < 0.03:
                self.publish_twist(0.0, 0.0, 0.0)
                self.get_logger().info(f"Reached Staging Pose: X={self.current_x:.3f}m, Y={self.current_y:.3f}m")
                self.transition(self.STATE_PRE_LIFT_ALIGN)

        # -------------------------------------------------------------
        # STATE 2: PRE_LIFT_ALIGN (PRE-RAISE FORK TO MATCH PALLET ENTRY SLOT)
        # -------------------------------------------------------------
        elif self.state == self.STATE_PRE_LIFT_ALIGN:
            self.publish_twist(0.0, 0.0, 0.0)
            # Raise fork to exact height matching the pallet runner slot
            self.set_lift(self.lift_insert_height)
            if elapsed > 1.5:
                self.get_logger().info(
                    f"Fork aligned to exact pallet slot height: {self.lift_insert_height*100:.2f}cm. "
                    "Starting Step 3: Slow fork insertion into pallet..."
                )
                self.transition(self.STATE_FORK_INSERT)

        # -------------------------------------------------------------
        # STATE 3: FORK_INSERT (Slow forward insertion into pallet slot)
        # -------------------------------------------------------------
        elif self.state == self.STATE_FORK_INSERT:
            self.set_lift(self.lift_insert_height)
            vx, vy, wz, dist, dyaw = self.compute_navigation_cmd(
                self.insert_x, self.target_y, self.target_yaw, max_v=0.08, max_w=0.4
            )
            vy = max(min(vy, 0.02), -0.02)  # minimize lateral drift
            self.publish_twist(vx, vy, wz)

            if self.current_x <= self.insert_x + 0.015 or elapsed > 6.0:
                self.publish_twist(0.0, 0.0, 0.0)
                self.get_logger().info(
                    f"Fork fully inserted under pallet at X={self.current_x:.3f}m. "
                    f"Lifting pallet to {self.lift_carry_height*100:.2f}cm..."
                )
                self.transition(self.STATE_LIFT_PALLET)

        # -------------------------------------------------------------
        # STATE 4: LIFT_PALLET (Elevate pallet off shelf surface)
        # -------------------------------------------------------------
        elif self.state == self.STATE_LIFT_PALLET:
            self.publish_twist(0.0, 0.0, 0.0)
            self.set_lift(self.lift_carry_height)
            if elapsed > 2.0:
                self.get_logger().info("Pallet elevated! Starting Step 5: Retracting fork out of rack...")
                self.transition(self.STATE_FORK_RETRACT)

        # -------------------------------------------------------------
        # STATE 5: FORK_RETRACT (Drive backward straight out of shelf)
        # -------------------------------------------------------------
        elif self.state == self.STATE_FORK_RETRACT:
            self.set_lift(self.lift_carry_height)
            vx, vy, wz, dist, dyaw = self.compute_navigation_cmd(
                self.staging_x, self.target_y, self.target_yaw, max_v=0.10, max_w=0.4
            )
            self.publish_twist(vx, vy, wz)

            if self.current_x >= self.staging_x - 0.02 or elapsed > 6.0:
                self.publish_twist(0.0, 0.0, 0.0)
                self.get_logger().info(f"Cleared shelf at X={self.current_x:.3f}m. Returning Home...")
                self.transition(self.STATE_NAV_RETURN)

        # -------------------------------------------------------------
        # STATE 6: NAV_RETURN (Return to starting pose)
        # -------------------------------------------------------------
        elif self.state == self.STATE_NAV_RETURN:
            self.set_lift(self.lift_carry_height)
            vx, vy, wz, dist, dyaw = self.compute_navigation_cmd(
                self.home_x, self.home_y, self.home_yaw, max_v=0.25, max_w=0.6
            )
            self.publish_twist(vx, vy, wz)

            if dist < 0.03 and dyaw < 0.04:
                self.publish_twist(0.0, 0.0, 0.0)
                self.get_logger().info(f"Arrived at Home Base: X={self.current_x:.3f}m, Y={self.current_y:.3f}m. Lowering Pallet...")
                self.transition(self.STATE_LOWER_PALLET)

        # -------------------------------------------------------------
        # STATE 7: LOWER_PALLET (Lower cargo at home base)
        # -------------------------------------------------------------
        elif self.state == self.STATE_LOWER_PALLET:
            self.publish_twist(0.0, 0.0, 0.0)
            self.set_lift(0.0)
            if elapsed > 2.0:
                self.get_logger().info("Pallet safely unloaded on the ground!")
                self.transition(self.STATE_COMPLETED)

        # -------------------------------------------------------------
        # STATE 8: COMPLETED
        # -------------------------------------------------------------
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
