#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
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
        self.declare_parameter('turbo_multiplier', 2.0)
        self.declare_parameter('lift_speed', 0.10)       # 0.10 m/s speed of lift
        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.18)

        # Mecanum Robot Geometry
        self.wheel_radius = 0.0487  # m
        self.lx = 0.1000            # Half wheelbase (m)
        self.ly = 0.1539            # Half track width (m)
        self.k_geom = self.lx + self.ly

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/lift_joint_cmd', 10)

        # Wheel velocity publishers for synchronized rotation
        self.wheel_fl_pub = self.create_publisher(Float64, '/wheel_fl_cmd_vel', 10)
        self.wheel_fr_pub = self.create_publisher(Float64, '/wheel_fr_cmd_vel', 10)
        self.wheel_rl_pub = self.create_publisher(Float64, '/wheel_rl_cmd_vel', 10)
        self.wheel_rr_pub = self.create_publisher(Float64, '/wheel_rr_cmd_vel', 10)

        # Subscriber to /joy
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Internal state
        self.current_lift_pos = 0.0
        self.was_moving = False
        self.latest_joy = None

        # Control loop at 20Hz (50ms interval)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Publish initial 0.0 lift command
        initial_msg = Float64()
        initial_msg.data = 0.0
        self.lift_pub.publish(initial_msg)

        self.get_logger().info('Robot0 Teleop Node started with active auto-brake, continuous lift & kinematic wheel rotation.')

    def joy_callback(self, msg: Joy):
        self.latest_joy = msg

    def publish_wheels(self, w_fl: float, w_fr: float, w_rl: float, w_rr: float):
        msg = Float64()
        msg.data = float(w_fl); self.wheel_fl_pub.publish(msg)
        msg.data = float(w_fr); self.wheel_fr_pub.publish(msg)
        msg.data = float(w_rl); self.wheel_rl_pub.publish(msg)
        msg.data = float(w_rr); self.wheel_rr_pub.publish(msg)

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
        axis_right_x = msg.axes[3] if len(msg.axes) > 3 else 0.0
        axis_dpad_y = msg.axes[7] if len(msg.axes) > 7 else 0.0

        # Turbo multiplier if RB is held
        multiplier = turbo_mult if (btn_rb == 1) else 1.0

        # -----------------------------------------------------------------
        # 1. Base Movement Control (/cmd_vel) & Mecanum Kinematic Wheels
        # -----------------------------------------------------------------
        twist = Twist()
        is_moving_now = False

        if is_enabled:
            # Translation: Left stick
            if abs(axis_left_y) > deadzone:
                twist.linear.x = axis_left_y * scale_x * multiplier
                is_moving_now = True

            if abs(axis_left_x) > deadzone:
                twist.linear.y = axis_left_x * scale_y * multiplier
                is_moving_now = True

            # In-place Rotation: Button X (Rotate Left) or Button B (Rotate Right)
            if btn_x == 1 and btn_b == 0:
                twist.angular.z = scale_ang * multiplier      # Turn Left (CCW)
                is_moving_now = True
            elif btn_b == 1 and btn_x == 0:
                twist.angular.z = -scale_ang * multiplier     # Turn Right (CW)
                is_moving_now = True
            elif abs(axis_right_x) > deadzone:
                twist.angular.z = axis_right_x * scale_ang * multiplier
                is_moving_now = True

        if is_moving_now:
            self.cmd_vel_pub.publish(twist)
            # Calculate Mecanum wheel angular velocities
            w_fl = (twist.linear.x - twist.linear.y - self.k_geom * twist.angular.z) / self.wheel_radius
            w_fr = (twist.linear.x + twist.linear.y + self.k_geom * twist.angular.z) / self.wheel_radius
            w_rl = (twist.linear.x + twist.linear.y - self.k_geom * twist.angular.z) / self.wheel_radius
            w_rr = (twist.linear.x - twist.linear.y + self.k_geom * twist.angular.z) / self.wheel_radius
            self.publish_wheels(w_fl, w_fr, w_rl, w_rr)
            self.was_moving = True
        else:
            if self.was_moving:
                self.cmd_vel_pub.publish(twist)
                self.publish_wheels(0.0, 0.0, 0.0, 0.0)
                self.was_moving = False

        # -----------------------------------------------------------------
        # 2. Lift Mechanism Control (/lift_joint_cmd)
        # -----------------------------------------------------------------
        step = lift_speed * self.timer_period

        lift_changed = False
        # Raise: Button Y or D-Pad Up
        if btn_y == 1 or axis_dpad_y > 0.5:
            new_pos = min(lift_max, self.current_lift_pos + step)
            if new_pos != self.current_lift_pos:
                self.current_lift_pos = new_pos
                lift_changed = True
        # Lower: Button A or D-Pad Down
        elif btn_a == 1 or axis_dpad_y < -0.5:
            new_pos = max(lift_min, self.current_lift_pos - step)
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
            node.publish_wheels(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
