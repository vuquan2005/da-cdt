# -*- coding: utf-8 -*-

import time
from typing import Optional
from .node import TreeNode, NodeStatus, Blackboard


class DecoratorNode(TreeNode):
    """Base class for decorators that modify the behavior of a single child node."""
    def __init__(self, name: str, child: TreeNode, blackboard: Optional[Blackboard] = None):
        super().__init__(name, blackboard)
        self.child: TreeNode = child
        self.child.parent = self
        if blackboard is not None:
            self.child.attach_blackboard(blackboard)

    def attach_blackboard(self, blackboard: Blackboard) -> None:
        super().attach_blackboard(blackboard)
        self.child.attach_blackboard(blackboard)

    def reset(self) -> None:
        super().reset()
        self.child.reset()


class Inverter(DecoratorNode):
    """Inverts SUCCESS to FAILURE and FAILURE to SUCCESS."""
    def update(self) -> NodeStatus:
        status = self.child.tick()
        if status == NodeStatus.SUCCESS:
            return NodeStatus.FAILURE
        if status == NodeStatus.FAILURE:
            return NodeStatus.SUCCESS
        return status


class ForceSuccessNode(DecoratorNode):
    """Always returns SUCCESS when child completes (regardless of SUCCESS or FAILURE)."""
    def update(self) -> NodeStatus:
        status = self.child.tick()
        if status == NodeStatus.RUNNING:
            return NodeStatus.RUNNING
        return NodeStatus.SUCCESS


class RetryNode(DecoratorNode):
    """Retries child execution up to max_attempts on FAILURE."""
    def __init__(self, name: str, child: TreeNode, max_attempts: int = 3, blackboard: Optional[Blackboard] = None):
        super().__init__(name, child, blackboard)
        self.max_attempts = max_attempts
        self.current_attempt = 0

    def initialise(self) -> None:
        self.current_attempt = 0

    def update(self) -> NodeStatus:
        while self.current_attempt < self.max_attempts:
            status = self.child.tick()
            if status == NodeStatus.SUCCESS:
                return NodeStatus.SUCCESS
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            else:
                self.current_attempt += 1
                self.child.reset()

        return NodeStatus.FAILURE


class TimeoutNode(DecoratorNode):
    """Fails if child takes longer than timeout_sec."""
    def __init__(self, name: str, child: TreeNode, timeout_sec: float, blackboard: Optional[Blackboard] = None):
        super().__init__(name, child, blackboard)
        self.timeout_sec = timeout_sec
        self.start_time = 0.0

    def initialise(self) -> None:
        self.start_time = time.time()

    def update(self) -> NodeStatus:
        if time.time() - self.start_time > self.timeout_sec:
            return NodeStatus.FAILURE
        return self.child.tick()
