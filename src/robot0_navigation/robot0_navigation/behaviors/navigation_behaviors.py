# -*- coding: utf-8 -*-

"""
Navigation Behavior Tree Action Nodes for Robot0.
Provides robust closed-loop motion control for 4WD Mecanum wheel mobile base:
- NavigateToPoseAction: Precise 2D pose navigation (X, Y, Yaw) with holonomic PID controller.
- NavigateThroughWaypointsAction: Smooth multi-waypoint path tracking with transit radii.
- LinearDriveAction: Precision relative linear displacement with active heading lock.
"""

import math
import time
from typing import Optional, Union, List
from ..behavior_tree.node import ActionNode, NodeStatus, Blackboard
from ..arena_coordinates import Pose2D, Pose3D


def normalize_angle(angle: float) -> float:
    """Normalizes an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(val: float, min_val: float, max_val: float) -> float:
    """Clamps a numeric value within [min_val, max_val]."""
    return max(min_val, min(max_val, val))


class NavigateToPoseAction(ActionNode):
    """
    Closed-Loop Holonomic Action Node to navigate robot to a target pose (x, y, yaw).
    Computes body-frame Cartesian errors and applies proportional velocity control with
    acceleration clamping, slow-down ramp, and heading alignment.
    """
    def __init__(
        self,
        name: str,
        target_pose: Union[Pose2D, Pose3D, str],
        pos_tolerance: float = 0.03,
        yaw_tolerance: float = 0.05,
        max_v: float = 0.25,
        max_w: float = 0.70,
        timeout_sec: float = 30.0,
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

        # Controller Gains
        self.kp_pos = 1.2 if not is_insert_mode else 0.8
        self.kp_yaw = 1.5 if not is_insert_mode else 1.0
        self.min_v = 0.03
        self.min_w = 0.06

        self.target_pose: Optional[Pose2D] = None
        self.start_time: float = 0.0

    def initialise(self) -> None:
        """Resolves target pose and initializes timer."""
        self.start_time = time.time()

        if isinstance(self.target_spec, str):
            pose_obj = self.blackboard.get(self.target_spec)
        else:
            pose_obj = self.target_spec

        if pose_obj is not None:
            yaw_val = getattr(pose_obj, 'yaw', 0.0)
            self.target_pose = Pose2D(x=float(pose_obj.x), y=float(pose_obj.y), yaw=float(yaw_val))
        else:
            self.target_pose = None
            ros_node = self.blackboard.get('ros_node')
            if ros_node:
                ros_node.get_logger().warn(
                    f"[BT] NavigateToPoseAction '{self.name}': Target spec '{self.target_spec}' could not be resolved!"
                )

    def update(self) -> NodeStatus:
        """Executes closed-loop holonomic control loop."""
        ros_node = self.blackboard.get('ros_node')
        current_x = self.blackboard.get('current_x')
        current_y = self.blackboard.get('current_y')
        current_yaw = self.blackboard.get('current_yaw')

        if current_x is None or current_y is None or current_yaw is None or ros_node is None or self.target_pose is None:
            return NodeStatus.RUNNING

        # 1. Check Timeout
        now = time.time()
        if now - self.start_time > self.timeout_sec:
            if ros_node:
                ros_node.get_logger().error(
                    f"[BT] NavigateToPoseAction '{self.name}': TIMEOUT ({self.timeout_sec}s) navigating to ({self.target_pose.x:.3f}, {self.target_pose.y:.3f})!"
                )
            ros_node.publish_twist(0.0, 0.0, 0.0)
            return NodeStatus.FAILURE

        # 2. Compute World-Frame & Body-Frame Errors
        dx_world = self.target_pose.x - float(current_x)
        dy_world = self.target_pose.y - float(current_y)
        dist_error = math.hypot(dx_world, dy_world)

        cos_yaw = math.cos(float(current_yaw))
        sin_yaw = math.sin(float(current_yaw))

        # Rotate world error vector into robot body frame
        ex_body = dx_world * cos_yaw + dy_world * sin_yaw
        ey_body = -dx_world * sin_yaw + dy_world * cos_yaw
        e_yaw = normalize_angle(self.target_pose.yaw - float(current_yaw))

        # 3. Check Success Criteria
        if dist_error <= self.pos_tolerance and abs(e_yaw) <= self.yaw_tolerance:
            ros_node.publish_twist(0.0, 0.0, 0.0)
            if ros_node:
                ros_node.get_logger().info(
                    f"[BT] NavigateToPoseAction '{self.name}': SUCCESS reached target "
                    f"({self.target_pose.x:.3f}, {self.target_pose.y:.3f}, yaw={math.degrees(self.target_pose.yaw):.1f}deg) "
                    f"with err={dist_error*1000.0:.1f}mm, yaw_err={math.degrees(abs(e_yaw)):.1f}deg."
                )
            return NodeStatus.SUCCESS

        # 4. Decoupled In-Place Rotation Check:
        # If heading is misaligned by more than 0.20 rad (~11.5 deg), rotate strictly in-place first!
        if abs(e_yaw) > 0.20 and not self.is_insert_mode:
            raw_wz = self.kp_yaw * e_yaw
            wz = clamp(raw_wz, -self.max_w, self.max_w)
            if abs(wz) < self.min_w:
                wz = math.copysign(self.min_w, e_yaw)
            ros_node.publish_twist(0.0, 0.0, wz)
            return NodeStatus.RUNNING

        # 5. Holonomic Velocity Calculations (Clean translation when aligned)
        if self.is_insert_mode:
            # Creep mode: Gentle linear insertion with tight lateral and angular limits
            max_insert_v = 0.08
            vx = clamp(self.kp_pos * ex_body, -max_insert_v, max_insert_v)
            vy = clamp(self.kp_pos * ey_body, -0.04, 0.04)
            wz = clamp(self.kp_yaw * e_yaw, -0.15, 0.15)
        else:
            # Standard navigation mode
            raw_vx = self.kp_pos * ex_body
            raw_vy = self.kp_pos * ey_body
            v_mag = math.hypot(raw_vx, raw_vy)

            if v_mag > self.max_v:
                scale = self.max_v / v_mag
                vx = raw_vx * scale
                vy = raw_vy * scale
            elif dist_error > self.pos_tolerance:
                creep_min = 0.015 if self.pos_tolerance < 0.01 else self.min_v
                if v_mag < creep_min:
                    scale = creep_min / max(1e-6, v_mag)
                    vx = raw_vx * scale
                    vy = raw_vy * scale
                else:
                    vx = raw_vx
                    vy = raw_vy
            else:
                vx = raw_vx
                vy = raw_vy

            raw_wz = self.kp_yaw * e_yaw
            wz = clamp(raw_wz, -0.30, 0.30)
            if abs(wz) < self.min_w and abs(e_yaw) > self.yaw_tolerance:
                wz = math.copysign(self.min_w, e_yaw)

        # In case position is close but yaw is still adjusting, reduce linear creep
        if dist_error <= self.pos_tolerance:
            vx = 0.0
            vy = 0.0

        ros_node.publish_twist(vx, vy, wz)
        return NodeStatus.RUNNING

    def terminate(self, new_status: NodeStatus) -> None:
        """Ensures robot stops when action terminates."""
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)


class LinearDriveAction(ActionNode):
    """
    Action Node to drive robot forward/backward or sideways by a relative distance.
    Uses odometry dead-reckoning with active heading stabilization lock.
    """
    def __init__(
        self,
        name: str,
        distance_meters: float,
        axis: str = 'x',
        speed: float = 0.10,
        tolerance: float = 0.015,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.distance_meters = distance_meters
        self.axis = axis.lower()
        self.speed = abs(speed)
        self.tolerance = tolerance

        self.target_dist: float = abs(distance_meters)
        self.sign: float = 1.0 if distance_meters >= 0 else -1.0
        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_yaw: Optional[float] = None
        self.start_time: float = 0.0
        self.timeout_sec: float = max(10.0, (self.target_dist / max(0.01, self.speed)) * 3.5)

    def initialise(self) -> None:
        """Captures start odometry reference pose."""
        self.start_x = self.blackboard.get('current_x')
        self.start_y = self.blackboard.get('current_y')
        self.start_yaw = self.blackboard.get('current_yaw')
        self.start_time = time.time()

    def update(self) -> NodeStatus:
        """Tracks traveled distance and commands linear velocity with heading lock."""
        ros_node = self.blackboard.get('ros_node')
        current_x = self.blackboard.get('current_x')
        current_y = self.blackboard.get('current_y')
        current_yaw = self.blackboard.get('current_yaw')

        if self.start_x is None or self.start_y is None or self.start_yaw is None:
            self.initialise()
            return NodeStatus.RUNNING

        if current_x is None or current_y is None or current_yaw is None or ros_node is None:
            return NodeStatus.RUNNING

        # 1. Check Timeout
        now = time.time()
        if now - self.start_time > self.timeout_sec:
            if ros_node:
                ros_node.get_logger().error(
                    f"[BT] LinearDriveAction '{self.name}': TIMEOUT ({self.timeout_sec:.1f}s) driving {self.distance_meters:.3f}m!"
                )
            ros_node.publish_twist(0.0, 0.0, 0.0)
            return NodeStatus.FAILURE

        # 2. Compute Traveled Distance
        traveled_dist = math.hypot(float(current_x) - self.start_x, float(current_y) - self.start_y)
        remaining_dist = self.target_dist - traveled_dist

        # 3. Check Success
        if remaining_dist <= self.tolerance:
            ros_node.publish_twist(0.0, 0.0, 0.0)
            if ros_node:
                ros_node.get_logger().info(
                    f"[BT] LinearDriveAction '{self.name}': SUCCESS traveled {traveled_dist:.3f}m (target: {self.distance_meters:.3f}m)."
                )
            return NodeStatus.SUCCESS

        # 4. Heading Stabilization Lock
        e_yaw = normalize_angle(self.start_yaw - float(current_yaw))
        wz = clamp(1.8 * e_yaw, -0.40, 0.40)

        # 5. Deceleration Ramp Near Goal
        if remaining_dist < 0.05:
            current_speed = max(0.03, self.speed * (remaining_dist / 0.05))
        else:
            current_speed = self.speed

        if self.axis == 'y':
            vx = 0.0
            vy = self.sign * current_speed
        else:
            vx = self.sign * current_speed
            vy = 0.0

        ros_node.publish_twist(vx, vy, wz)
        return NodeStatus.RUNNING

    def terminate(self, new_status: NodeStatus) -> None:
        """Stops robot when action finishes."""
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)


class NavigateThroughWaypointsAction(ActionNode):
    """
    Action Node to sequentially traverse a list of waypoints with continuous curvature
    transit radii for intermediate waypoints and precise docking for the final destination.
    """
    def __init__(
        self,
        name: str,
        waypoints_spec: Union[List[Pose2D], str],
        pos_tolerance: float = 0.03,
        yaw_tolerance: float = 0.05,
        transit_radius: float = 0.08,
        max_v: float = 0.25,
        max_w: float = 0.70,
        timeout_per_wp: float = 20.0,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.waypoints_spec = waypoints_spec
        self.pos_tolerance = pos_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.transit_radius = transit_radius
        self.max_v = max_v
        self.max_w = max_w
        self.timeout_per_wp = timeout_per_wp

        # Controller Gains
        self.kp_pos = 1.2
        self.kp_yaw = 1.5
        self.min_v = 0.04
        self.min_w = 0.06

        self.waypoints: List[Pose2D] = []
        self.current_idx: int = 0
        self.wp_start_time: float = 0.0

    def initialise(self) -> None:
        """Loads waypoints and resets index."""
        self.current_idx = 0
        self.wp_start_time = time.time()

        if isinstance(self.waypoints_spec, str):
            wps = self.blackboard.get(self.waypoints_spec, [])
            self.waypoints = list(wps) if wps else []
        else:
            self.waypoints = list(self.waypoints_spec)

        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.get_logger().info(
                f"[BT] NavigateThroughWaypointsAction '{self.name}': Initialized with {len(self.waypoints)} waypoints."
            )

    def update(self) -> NodeStatus:
        """Executes waypoint path traversal."""
        ros_node = self.blackboard.get('ros_node')
        current_x = self.blackboard.get('current_x')
        current_y = self.blackboard.get('current_y')
        current_yaw = self.blackboard.get('current_yaw')

        if not self.waypoints:
            # Empty route -> immediate success
            return NodeStatus.SUCCESS

        if current_x is None or current_y is None or current_yaw is None or ros_node is None:
            return NodeStatus.RUNNING

        # 1. Check Per-Waypoint Timeout
        now = time.time()
        if now - self.wp_start_time > self.timeout_per_wp:
            if ros_node:
                ros_node.get_logger().error(
                    f"[BT] NavigateThroughWaypointsAction '{self.name}': TIMEOUT ({self.timeout_per_wp}s) on waypoint #{self.current_idx + 1}!"
                )
            ros_node.publish_twist(0.0, 0.0, 0.0)
            return NodeStatus.FAILURE

        num_wps = len(self.waypoints)
        target_wp = self.waypoints[self.current_idx]
        target_yaw = getattr(target_wp, 'yaw', 0.0)
        is_last_wp = (self.current_idx == num_wps - 1)

        # 2. Compute Distance & Frame Errors
        dx_world = target_wp.x - float(current_x)
        dy_world = target_wp.y - float(current_y)
        dist_error = math.hypot(dx_world, dy_world)

        cos_yaw = math.cos(float(current_yaw))
        sin_yaw = math.sin(float(current_yaw))

        ex_body = dx_world * cos_yaw + dy_world * sin_yaw
        ey_body = -dx_world * sin_yaw + dy_world * cos_yaw
        e_yaw = normalize_angle(target_yaw - float(current_yaw))

        # 3. Intermediate Waypoint Switching Logic
        if not is_last_wp:
            if dist_error <= self.transit_radius:
                # Reached transit threshold -> advance to next waypoint without stopping
                self.current_idx += 1
                self.wp_start_time = now
                if ros_node:
                    ros_node.get_logger().info(
                        f"[BT] Reached intermediate WP #{self.current_idx} -> Switching to #{self.current_idx + 1}/{num_wps}"
                    )
                return NodeStatus.RUNNING

        # 4. Final Destination Stopping Logic
        else:
            if dist_error <= self.pos_tolerance and abs(e_yaw) <= self.yaw_tolerance:
                ros_node.publish_twist(0.0, 0.0, 0.0)
                if ros_node:
                    ros_node.get_logger().info(
                        f"[BT] NavigateThroughWaypointsAction '{self.name}': SUCCESS reached final destination!"
                    )
                return NodeStatus.SUCCESS

        # 5. Decoupled In-Place Rotation Check:
        # If heading angle error is significant (> 0.20 rad ~ 11.5 deg), rotate cleanly in place first!
        if abs(e_yaw) > 0.20:
            raw_wz = self.kp_yaw * e_yaw
            wz = clamp(raw_wz, -self.max_w, self.max_w)
            if abs(wz) < self.min_w:
                wz = math.copysign(self.min_w, e_yaw)
            ros_node.publish_twist(0.0, 0.0, wz)
            return NodeStatus.RUNNING

        # 6. Holonomic Velocity Controller (Clean translation when aligned)
        raw_vx = self.kp_pos * ex_body
        raw_vy = self.kp_pos * ey_body
        v_mag = math.hypot(raw_vx, raw_vy)

        if v_mag > self.max_v:
            scale = self.max_v / v_mag
            vx = raw_vx * scale
            vy = raw_vy * scale
        elif v_mag < self.min_v and dist_error > self.pos_tolerance:
            scale = self.min_v / max(1e-6, v_mag)
            vx = raw_vx * scale
            vy = raw_vy * scale
        else:
            vx = raw_vx
            vy = raw_vy

        # Control yaw (fine lock)
        raw_wz = self.kp_yaw * e_yaw
        wz = clamp(raw_wz, -0.30, 0.30)
        if is_last_wp:
            if abs(wz) < self.min_w and abs(e_yaw) > self.yaw_tolerance:
                wz = math.copysign(self.min_w, e_yaw)

            if dist_error <= self.pos_tolerance:
                vx = 0.0
                vy = 0.0

        ros_node.publish_twist(vx, vy, wz)
        return NodeStatus.RUNNING

    def terminate(self, new_status: NodeStatus) -> None:
        """Stops robot when action finishes."""
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)


