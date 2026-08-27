# -*- coding: utf-8 -*-

import time
import math
from typing import Optional, Union
from ..behavior_tree.node import ActionNode, NodeStatus, Blackboard
from ..arena_coordinates import Pose2D, Pose3D


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class NavigateToPoseAction(ActionNode):
    """
    Mecanum Omnidirectional Navigation Controller towards a 2D Target Pose (x, y, yaw).
    Smoothly drives robot with proportional position & heading control.
    """
    def __init__(
        self,
        name: str,
        target_pose: Union[Pose2D, Pose3D, str],
        pos_tolerance: float = 0.025,
        yaw_tolerance: float = 0.035,
        max_v: float = 0.25,
        max_w: float = 0.70,
        timeout_sec: float = 25.0,
        is_insert_mode: bool = False,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.target_spec = target_pose
        self.pos_tolerance = pos_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.max_v = max_v
        self.max_w = max_w
        self.timeout_sec = timeout_sec
        self.is_insert_mode = is_insert_mode

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.start_time = 0.0

    def initialise(self) -> None:
        self.start_time = time.time()

        if isinstance(self.target_spec, str):
            pose_obj = self.blackboard.get(self.target_spec)
        else:
            pose_obj = self.target_spec

        if pose_obj is not None:
            self.target_x = float(pose_obj.x)
            self.target_y = float(pose_obj.y)
            self.target_yaw = float(pose_obj.yaw)
        else:
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_yaw = 0.0

    def update(self) -> NodeStatus:
        current_x = self.blackboard.get('current_x')
        current_y = self.blackboard.get('current_y')
        current_yaw = self.blackboard.get('current_yaw')
        ros_node = self.blackboard.get('ros_node')

        if current_x is None or ros_node is None:
            return NodeStatus.RUNNING

        if time.time() - self.start_time > self.timeout_sec:
            ros_node.publish_twist(0.0, 0.0, 0.0)
            ros_node.get_logger().warn(f"[BT] NavigateToPoseAction '{self.name}' completed on timeout safeguard.")
            return NodeStatus.SUCCESS if self.is_insert_mode else NodeStatus.FAILURE

        dx_world = self.target_x - current_x
        dy_world = self.target_y - current_y
        dist = math.hypot(dx_world, dy_world)
        dyaw = normalize_angle(self.target_yaw - current_yaw)

        # In insert mode (facing west at yaw=pi): check if fork is sufficiently inserted
        if self.is_insert_mode:
            if current_x <= self.target_x + 0.015 and abs(dy_world) <= 0.035 and abs(dyaw) <= 0.06:
                ros_node.publish_twist(0.0, 0.0, 0.0)
                ros_node.get_logger().info(f"[BT] Insert completed at X={current_x:.3f}m, Y={current_y:.3f}m")
                return NodeStatus.SUCCESS

        # Check goal arrival
        if dist <= self.pos_tolerance and abs(dyaw) <= self.yaw_tolerance:
            ros_node.publish_twist(0.0, 0.0, 0.0)
            return NodeStatus.SUCCESS

        # Transform world error into robot body frame
        cos_y = math.cos(current_yaw)
        sin_y = math.sin(current_yaw)
        dx_body = dx_world * cos_y + dy_world * sin_y
        dy_body = -dx_world * sin_y + dy_world * cos_y

        # Proportional controller gains
        kp_pos = 1.35
        kp_yaw = 1.60

        vx = max(min(kp_pos * dx_body, self.max_v), -self.max_v)
        vy = max(min(kp_pos * dy_body, self.max_v), -self.max_v)
        wz = max(min(kp_yaw * dyaw, self.max_w), -self.max_w)

        # Prevent stall with minimum creeping velocity
        if dist > self.pos_tolerance:
            v_mag = math.hypot(vx, vy)
            min_v = 0.04
            if v_mag < min_v and v_mag > 1e-4:
                scale = min_v / v_mag
                vx *= scale
                vy *= scale

        ros_node.publish_twist(vx, vy, wz)
        return NodeStatus.RUNNING

    def terminate(self, new_status: NodeStatus) -> None:
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)


class LinearDriveAction(ActionNode):
    """
    Drives robot body forward/backward (along body X) or sideways (along body Y) by a relative distance.
    Ideal for delicate fork insertion, pallet extraction, and drop-off backoff.
    """
    def __init__(
        self,
        name: str,
        distance_meters: float,
        axis: str = 'x',
        speed: float = 0.08,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.distance_meters = distance_meters
        self.axis = axis.lower()
        self.speed = abs(speed)
        self.direction = 1.0 if distance_meters >= 0 else -1.0
        self.target_dist = abs(distance_meters)

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_time = 0.0
        self.traveled = 0.0

    def initialise(self) -> None:
        self.start_time = time.time()
        self.traveled = 0.0
        self.start_x = float(self.blackboard.get('current_x', 0.0))
        self.start_y = float(self.blackboard.get('current_y', 0.0))

    def update(self) -> NodeStatus:
        current_x = self.blackboard.get('current_x')
        current_y = self.blackboard.get('current_y')
        ros_node = self.blackboard.get('ros_node')

        if current_x is None or ros_node is None:
            return NodeStatus.RUNNING

        self.traveled = math.hypot(current_x - self.start_x, current_y - self.start_y)

        # Safety timeout
        expected_time = (self.target_dist / max(self.speed, 0.01)) * 2.5 + 2.0
        if time.time() - self.start_time > expected_time:
            ros_node.publish_twist(0.0, 0.0, 0.0)
            return NodeStatus.SUCCESS

        if self.traveled >= self.target_dist - 0.005:
            ros_node.publish_twist(0.0, 0.0, 0.0)
            return NodeStatus.SUCCESS

        # Command velocity in robot frame
        v_cmd = self.direction * self.speed
        if self.axis == 'y':
            ros_node.publish_twist(0.0, v_cmd, 0.0)
        else:
            ros_node.publish_twist(v_cmd, 0.0, 0.0)

        return NodeStatus.RUNNING

    def terminate(self, new_status: NodeStatus) -> None:
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)
