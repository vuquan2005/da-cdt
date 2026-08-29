# -*- coding: utf-8 -*-

import time
from typing import Optional, Any
from ..behavior_tree.node import ActionNode, ConditionNode, NodeStatus, Blackboard


class LogMessageAction(ActionNode):
    """Logs an informational message to ROS logger and Blackboard."""
    def __init__(self, name: str, message: str, level: str = 'info', blackboard: Optional[Blackboard] = None):
        super().__init__(name, blackboard)
        self.message = message
        self.level = level.lower()

    def update(self) -> NodeStatus:
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            logger = ros_node.get_logger()
            if self.level == 'warn':
                logger.warn(f'[BT] {self.message}')
            elif self.level == 'error':
                logger.error(f'[BT] {self.message}')
            else:
                logger.info(f'[BT] {self.message}')
        else:
            print(f'[BT LOG {self.level.upper()}] {self.message}')
        return NodeStatus.SUCCESS


class WaitAction(ActionNode):
    """Waits for a given duration in seconds before returning SUCCESS."""
    def __init__(self, name: str, duration_sec: float, blackboard: Optional[Blackboard] = None):
        super().__init__(name, blackboard)
        self.duration_sec = duration_sec
        self.start_time = 0.0

    def initialise(self) -> None:
        self.start_time = time.time()

    def update(self) -> NodeStatus:
        if time.time() - self.start_time >= self.duration_sec:
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING


class WaitForOdometryCondition(ActionNode):
    """Waits asynchronously until valid Odometry is received by the robot."""
    def __init__(self, name: str, timeout_sec: float = 10.0, blackboard: Optional[Blackboard] = None):
        super().__init__(name, blackboard)
        self.timeout_sec = timeout_sec
        self.start_time = 0.0

    def initialise(self) -> None:
        self.start_time = time.time()

    def update(self) -> NodeStatus:
        current_x = self.blackboard.get('current_x')
        if current_x is not None:
            return NodeStatus.SUCCESS
        if time.time() - self.start_time > self.timeout_sec:
            ros_node = self.blackboard.get('ros_node')
            if ros_node:
                ros_node.get_logger().error(f"[BT] WaitForOdometry '{self.name}': TIMEOUT waiting for /odom!")
            return NodeStatus.FAILURE
        return NodeStatus.RUNNING


class SetBlackboardValueAction(ActionNode):
    """Sets a key-value pair in the shared Blackboard."""
    def __init__(self, name: str, key: str, value: Any, blackboard: Optional[Blackboard] = None):
        super().__init__(name, blackboard)
        self.key = key
        self.value = value

    def update(self) -> NodeStatus:
        self.blackboard.set(self.key, self.value)
        return NodeStatus.SUCCESS
