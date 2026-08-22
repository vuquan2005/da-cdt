#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class Robot0Teleop(Node):
    def __init__(self):
        super().__init__('robot0_teleop')

        # Declare parameters with default values
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('scale_linear_x', 0.5)
        self.declare_parameter('scale_linear_y', 0.5)
        self.declare_parameter('scale_angular_z', 1.2)
        self.declare_parameter('turbo_multiplier', 2.0)
        self.declare_parameter('lift_step', 0.005)      # Step per joy message (~0.1m/s at 20Hz)
        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.18)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, '/lift_position_controller/commands', 10)

        # Subscriber to /joy
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Internal state
        self.current_lift_pos = 0.0
        self.was_moving = False
        self.last_lift_cmd_sent = -1.0

        # Timer to maintain lift position publication at 10Hz if needed
        self.get_logger().info('Robot0 Teleop Node initialized successfully.')

    def joy_callback(self, msg: Joy):
        deadzone = self.get_parameter('deadzone').value
        scale_x = self.get_parameter('scale_linear_x').value
        scale_y = self.get_parameter('scale_linear_y').value
        scale_ang = self.get_parameter('scale_angular_z').value
        turbo_mult = self.get_parameter('turbo_multiplier').value
        lift_step = self.get_parameter('lift_step').value
        lift_min = self.get_parameter('lift_min').value
        lift_max = self.get_parameter('lift_max').value

        # Button mappings (Xbox / standard controller)
        # buttons: [A(0), B(1), X(2), Y(3), LB(4), RB(5), Back(6), Start(7), ...]
        # axes: [LeftX(0), LeftY(1), LT(2), RightX(3), RightY(4), RT(5), DpadX(6), DpadY(7)]
        btn_a = msg.buttons[0] if len(msg.buttons) > 0 else 0
        btn_b = msg.buttons[1] if len(msg.buttons) > 1 else 0
        btn_x = msg.buttons[2] if len(msg.buttons) > 2 else 0
        btn_y = msg.buttons[3] if len(msg.buttons) > 3 else 0
        btn_lb = msg.buttons[4] if len(msg.buttons) > 4 else 0
        btn_rb = msg.buttons[5] if len(msg.buttons) > 5 else 0

        axis_left_x = msg.axes[0] if len(msg.axes) > 0 else 0.0
        axis_left_y = msg.axes[1] if len(msg.axes) > 1 else 0.0
        axis_dpad_y = msg.axes[7] if len(msg.axes) > 7 else 0.0

        # Check enable / deadman button
        is_turbo = (btn_rb == 1)
        is_enabled = (btn_lb == 1) or is_turbo

        # -------------------------------------------------------------
        # 1. Base Movement Control (/cmd_vel)
        # -------------------------------------------------------------
        twist = Twist()

        if is_enabled:
            multiplier = turbo_mult if is_turbo else 1.0

            # Linear X (Forward/Backward)
            if abs(axis_left_y) > deadzone:
                twist.linear.x = axis_left_y * scale_x * multiplier
            else:
                twist.linear.x = 0.0

            # Linear Y (Strafe Left/Right for Omnidirectional Mecanum)
            if abs(axis_left_x) > deadzone:
                twist.linear.y = axis_left_x * scale_y * multiplier
            else:
                twist.linear.y = 0.0

            # Angular Z (Rotate Left with X, Rotate Right with B)
            if btn_x == 1 and btn_b == 0:
                twist.angular.z = scale_ang * multiplier      # Rotate Left (CCW)
            elif btn_b == 1 and btn_x == 0:
                twist.angular.z = -scale_ang * multiplier     # Rotate Right (CW)
            else:
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            self.was_moving = True
        else:
            # Active Brake / Stop: When deadman button is not held, publish 0 to stop Gazebo immediately
            if self.was_moving:
                self.cmd_vel_pub.publish(twist)
                self.was_moving = False

        # -------------------------------------------------------------
        # 2. Lift Mechanism Control (/lift_position_controller/commands)
        # -------------------------------------------------------------
        # Allow lift adjustment when LB is held or D-pad/Y/A is pressed
        lift_changed = False

        # Raise: Y button or D-pad Up
        if btn_y == 1 or axis_dpad_y > 0.5:
            self.current_lift_pos = min(lift_max, self.current_lift_pos + lift_step)
            lift_changed = True
        # Lower: A button or D-pad Down
        elif btn_a == 1 or axis_dpad_y < -0.5:
            self.current_lift_pos = max(lift_min, self.current_lift_pos - lift_step)
            lift_changed = True

        if lift_changed or abs(self.current_lift_pos - self.last_lift_cmd_sent) > 1e-4:
            lift_msg = Float64MultiArray()
            lift_msg.data = [float(self.current_lift_pos)]
            self.lift_pub.publish(lift_msg)
            self.last_lift_cmd_sent = self.current_lift_pos


def main(args=None):
    rclpy.init(args=args)
    node = Robot0Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a final zero twist before shutdown
        try:
            stop_twist = Twist()
            node.cmd_vel_pub.publish(stop_twist)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
