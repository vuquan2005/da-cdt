# -*- coding: utf-8 -*-

"""
Navigation Behavior Tree Action Nodes (Skeleton / Template).
Provides abstract/skeleton action nodes for robot navigation:
- NavigateToPoseAction: Navigates to a 2D/3D target pose.
- LinearDriveAction: Moves robot straight forward/backward/sideways.
- NavigateThroughWaypointsAction: Sequentially traverses a waypoint path.
"""

from typing import Optional, Union, List
from ..behavior_tree.node import ActionNode, NodeStatus, Blackboard
from ..arena_coordinates import Pose2D, Pose3D


class NavigateToPoseAction(ActionNode):
    """
    Skeleton Action Node to navigate robot to a target pose (x, y, yaw).
    Implement your custom motion controller / planner logic in update().
    """
    def __init__(
        self,
        name: str,
        target_pose: Union[Pose2D, Pose3D, str],
        pos_tolerance: float = 0.05,
        yaw_tolerance: float = 0.08,
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

        self.target_pose: Optional[Pose2D] = None

    def initialise(self) -> None:
        """Called once when node starts executing."""
        if isinstance(self.target_spec, str):
            pose_obj = self.blackboard.get(self.target_spec)
        else:
            pose_obj = self.target_spec

        if pose_obj is not None:
            self.target_pose = Pose2D(x=float(pose_obj.x), y=float(pose_obj.y), yaw=float(pose_obj.yaw))
        else:
            self.target_pose = None

    def update(self) -> NodeStatus:
        """
        Called every tree tick.
        TODO: Implement your control algorithm here (e.g. PID, Pure Pursuit, Nav2 client).
        Access current pose from blackboard ('current_x', 'current_y', 'current_yaw').
        Send velocity command via blackboard.get('ros_node').publish_twist(vx, vy, wz).
        """
        ros_node = self.blackboard.get('ros_node')
        current_x = self.blackboard.get('current_x')
        current_y = self.blackboard.get('current_y')
        current_yaw = self.blackboard.get('current_yaw')

        if current_x is None or ros_node is None or self.target_pose is None:
            return NodeStatus.RUNNING

        # Skeleton placeholder: returns SUCCESS immediately for structure testing
        ros_node.publish_twist(0.0, 0.0, 0.0)
        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        """Called when node finishes or is interrupted."""
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)


class LinearDriveAction(ActionNode):
    """
    Skeleton Action Node to drive robot forward/backward or sideways by a relative distance.
    """
    def __init__(
        self,
        name: str,
        distance_meters: float,
        axis: str = 'x',
        speed: float = 0.10,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.distance_meters = distance_meters
        self.axis = axis.lower()
        self.speed = abs(speed)

    def initialise(self) -> None:
        """Called once when node starts executing."""
        pass

    def update(self) -> NodeStatus:
        """
        Called every tree tick.
        TODO: Implement linear open-loop or closed-loop displacement control.
        """
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)
        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)


class NavigateThroughWaypointsAction(ActionNode):
    """
    Skeleton Action Node to navigate sequentially through a list of waypoints.
    """
    def __init__(
        self,
        name: str,
        waypoints_spec: Union[List[Pose2D], str],
        pos_tolerance: float = 0.05,
        yaw_tolerance: float = 0.08,
        max_v: float = 0.25,
        max_w: float = 0.70,
        timeout_per_wp: float = 15.0,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.waypoints_spec = waypoints_spec
        self.pos_tolerance = pos_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.max_v = max_v
        self.max_w = max_w
        self.timeout_per_wp = timeout_per_wp

        self.waypoints: List[Pose2D] = []
        self.current_idx: int = 0

    def initialise(self) -> None:
        """Called once when node starts executing."""
        self.current_idx = 0
        if isinstance(self.waypoints_spec, str):
            wps = self.blackboard.get(self.waypoints_spec, [])
            self.waypoints = list(wps) if wps else []
        else:
            self.waypoints = list(self.waypoints_spec)

    def update(self) -> NodeStatus:
        """
        Called every tree tick.
        TODO: Implement waypoint follower / trajectory tracker.
        """
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)
        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.publish_twist(0.0, 0.0, 0.0)

