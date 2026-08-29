#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Robot0Teleop(Node):
    def __init__(self):
        super().__init__('robot0_teleop')

        # Parameters
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('require_enable_button', True)
        self.declare_parameter('enable_button', 4)       # Button LB (index 4)
        self.declare_parameter('scale_linear_x', 0.5)
        self.declare_parameter('scale_linear_y', 0.5)
        self.declare_parameter('scale_angular_z', 1.2)
        self.declare_parameter('turbo_multiplier', 3.0)
        self.declare_parameter('lift_speed', 0.10)       # 0.10 m/s speed of lift
        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.20)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/lift_joint_cmd', 10)

        # Subscriber to /joy and /joint_states
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Internal state
        self.current_lift_pos = 0.0
        self.actual_lift_pos = 0.0
        self.was_moving = False
        self.latest_joy = None
        self.lt_calibrated = False
        self.rt_calibrated = False

        # Control loop at 20Hz (50ms interval)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Publish initial 0.0 lift command
        initial_msg = Float64()
        initial_msg.data = 0.0
        self.lift_pub.publish(initial_msg)

        self.get_logger().info('Robot0 Teleop Node started with active auto-brake, continuous lift & analog LT/RT turbo.')

    def joy_callback(self, msg: Joy):
        self.latest_joy = msg

    def joint_state_callback(self, msg: JointState):
        if 'lift_arm_joint' in msg.name:
            idx = msg.name.index('lift_arm_joint')
            if len(msg.position) > idx:
                self.actual_lift_pos = float(msg.position[idx])

    def control_loop(self):
        if self.latest_joy is None:
            return

        msg = self.latest_joy
        deadzone = self.get_parameter('deadzone').value
        require_enable = self.get_parameter('require_enable_button').value
        enable_btn_idx = self.get_parameter('enable_button').value
        scale_x = self.get_parameter('scale_linear_x').value
        scale_y = self.get_parameter('scale_linear_y').value
        scale_ang = self.get_parameter('scale_angular_z').value
        turbo_mult = self.get_parameter('turbo_multiplier').value
        lift_speed = self.get_parameter('lift_speed').value
        lift_min = self.get_parameter('lift_min').value
        lift_max = self.get_parameter('lift_max').value

        # Buttons: A(0), B(1), X(2), Y(3), LB(4), RB(5), Back(6), Start(7)
        btn_a = msg.buttons[0] if len(msg.buttons) > 0 else 0
        btn_b = msg.buttons[1] if len(msg.buttons) > 1 else 0
        btn_x = msg.buttons[2] if len(msg.buttons) > 2 else 0
        btn_y = msg.buttons[3] if len(msg.buttons) > 3 else 0
        btn_lb = msg.buttons[enable_btn_idx] if len(msg.buttons) > enable_btn_idx else 0
        btn_rb = msg.buttons[5] if len(msg.buttons) > 5 else 0

        # Enable/Deadman check
        is_enabled = (btn_lb == 1) if require_enable else True

        # Axes: LeftX(0), LeftY(1), LT(2), RightX(3), RightY(4), RT(5), DpadX(6), DpadY(7)
        axis_left_x = msg.axes[0] if len(msg.axes) > 0 else 0.0
        axis_left_y = msg.axes[1] if len(msg.axes) > 1 else 0.0
        axis_lt = msg.axes[2] if len(msg.axes) > 2 else 1.0
        axis_right_x = msg.axes[3] if len(msg.axes) > 3 else 0.0
        axis_rt = msg.axes[5] if len(msg.axes) > 5 else 1.0

        # -----------------------------------------------------------------
        # 1. Analog Triggers Processing (LT for Linear Gain, RT for Angular Gain)
        # -----------------------------------------------------------------
        # LT Trigger: Gain = 1.0 (unpressed), 0.5 (light press) -> 3.0 (full travel)
        if not self.lt_calibrated:
            if abs(axis_lt - 1.0) < 0.1 or abs(axis_lt) > 0.05:
                self.lt_calibrated = True

        linear_multiplier = 1.0
        if self.lt_calibrated:
            lt_depth = max(0.0, min(1.0, (1.0 - axis_lt) / 2.0))
            if lt_depth > deadzone:
                u_lt = (lt_depth - deadzone) / (1.0 - deadzone)
                linear_multiplier = 0.5 + (turbo_mult - 0.5) * u_lt

        # RT Trigger: Gain = 1.0 (unpressed) -> 3.0 (full travel) for self-rotation
        if not self.rt_calibrated:
            if abs(axis_rt - 1.0) < 0.1 or abs(axis_rt) > 0.05:
                self.rt_calibrated = True

        ang_multiplier = 1.0
        if self.rt_calibrated:
            rt_depth = max(0.0, min(1.0, (1.0 - axis_rt) / 2.0))
            if rt_depth > deadzone:
                u_rt = (rt_depth - deadzone) / (1.0 - deadzone)
                ang_multiplier = 1.0 + (turbo_mult - 1.0) * u_rt

        # Digital RB button override (applies turbo_mult to both linear and angular)
        if btn_rb == 1:
            linear_multiplier = max(linear_multiplier, turbo_mult)
            ang_multiplier = max(ang_multiplier, turbo_mult)

        # -----------------------------------------------------------------
        # 2. Base Movement Control (/cmd_vel)
        # -----------------------------------------------------------------
        twist = Twist()
        is_moving_now = False

        if is_enabled:
            # Translation: Left stick
            if abs(axis_left_y) > deadzone:
                twist.linear.x = axis_left_y * scale_x * linear_multiplier
                is_moving_now = True

            if abs(axis_left_x) > deadzone:
                twist.linear.y = axis_left_x * scale_y * linear_multiplier
                is_moving_now = True

            # Rotation (Yaw): Right stick
            if abs(axis_right_x) > deadzone:
                twist.angular.z = axis_right_x * scale_ang * ang_multiplier
                is_moving_now = True

        if is_moving_now:
            self.cmd_vel_pub.publish(twist)
            self.was_moving = True
        else:
            if self.was_moving:
                self.cmd_vel_pub.publish(twist)
                self.was_moving = False

        # -----------------------------------------------------------------
        # 3. Lift Mechanism Control (/lift_joint_cmd)
        # -----------------------------------------------------------------
        step = lift_speed * self.timer_period

        lift_changed = False
        # Sync with actual position when not actively commanding
        if btn_y == 0 and btn_a == 0 and abs(self.current_lift_pos - self.actual_lift_pos) > 0.002:
            self.current_lift_pos = self.actual_lift_pos

        # Raise: Button Y
        if btn_y == 1:
            base_ref = self.actual_lift_pos if abs(self.current_lift_pos - self.actual_lift_pos) > 0.01 else self.current_lift_pos
            new_pos = min(lift_max, base_ref + step)
            if new_pos != self.current_lift_pos:
                self.current_lift_pos = new_pos
                lift_changed = True
        # Lower: Button A
        elif btn_a == 1:
            base_ref = self.actual_lift_pos if abs(self.current_lift_pos - self.actual_lift_pos) > 0.01 else self.current_lift_pos
            new_pos = max(lift_min, base_ref - step)
            if new_pos != self.current_lift_pos:
                self.current_lift_pos = new_pos
                lift_changed = True

        if lift_changed:
            lift_msg = Float64()
            lift_msg.data = float(self.current_lift_pos)
            self.lift_pub.publish(lift_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Robot0Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop_twist = Twist()
            node.cmd_vel_pub.publish(stop_twist)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
