# -*- coding: utf-8 -*-

from typing import List, Optional
from .node import TreeNode, NodeStatus, Blackboard


class CompositeNode(TreeNode):
    """Base class for composite nodes that contain multiple child nodes."""
    def __init__(self, name: str, children: Optional[List[TreeNode]] = None, blackboard: Optional[Blackboard] = None):
        super().__init__(name, blackboard)
        self.children: List[TreeNode] = children if children is not None else []
        for child in self.children:
            child.parent = self
            if blackboard is not None:
                child.attach_blackboard(blackboard)

    def add_child(self, child: TreeNode) -> 'CompositeNode':
        child.parent = self
        child.attach_blackboard(self.blackboard)
        self.children.append(child)
        return self

    def attach_blackboard(self, blackboard: Blackboard) -> None:
        super().attach_blackboard(blackboard)
        for child in self.children:
            child.attach_blackboard(blackboard)

    def reset(self) -> None:
        super().reset()
        for child in self.children:
            child.reset()


class Sequence(CompositeNode):
    """
    Sequence Composite (AND Logic):
    Ticks children sequentially from left to right.
    - If a child returns RUNNING -> Sequence returns RUNNING.
    - If a child returns FAILURE -> Sequence returns FAILURE immediately.
    - If all children return SUCCESS -> Sequence returns SUCCESS.
    """
    def __init__(self, name: str, children: Optional[List[TreeNode]] = None, blackboard: Optional[Blackboard] = None):
        super().__init__(name, children, blackboard)
        self._current_child_idx: int = 0

    def initialise(self) -> None:
        self._current_child_idx = 0

    def update(self) -> NodeStatus:
        while self._current_child_idx < len(self.children):
            current_child = self.children[self._current_child_idx]
            status = current_child.tick()

            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                return NodeStatus.FAILURE
            elif status == NodeStatus.SUCCESS:
                self._current_child_idx += 1
            else:
                return NodeStatus.FAILURE

        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        if new_status != NodeStatus.RUNNING:
            self._current_child_idx = 0


class Selector(CompositeNode):
    """
    Selector / Fallback Composite (OR Logic):
    Ticks children sequentially from left to right.
    - If a child returns SUCCESS -> Selector returns SUCCESS immediately.
    - If a child returns RUNNING -> Selector returns RUNNING.
    - If all children return FAILURE -> Selector returns FAILURE.
    """
    def __init__(self, name: str, children: Optional[List[TreeNode]] = None, blackboard: Optional[Blackboard] = None):
        super().__init__(name, children, blackboard)
        self._current_child_idx: int = 0

    def initialise(self) -> None:
        self._current_child_idx = 0

    def update(self) -> NodeStatus:
        while self._current_child_idx < len(self.children):
            current_child = self.children[self._current_child_idx]
            status = current_child.tick()

            if status == NodeStatus.SUCCESS:
                return NodeStatus.SUCCESS
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                self._current_child_idx += 1
            else:
                self._current_child_idx += 1

        return NodeStatus.FAILURE

    def terminate(self, new_status: NodeStatus) -> None:
        if new_status != NodeStatus.RUNNING:
            self._current_child_idx = 0


class Parallel(CompositeNode):
    """
    Parallel Composite:
    Ticks ALL children concurrently in each tick cycle.
    """
    def __init__(
        self,
        name: str,
        success_threshold: int = -1,
        failure_threshold: int = 1,
        children: Optional[List[TreeNode]] = None,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, children, blackboard)
        self.success_threshold = success_threshold
        self.failure_threshold = failure_threshold

    def update(self) -> NodeStatus:
        num_success = 0
        num_failure = 0
        num_running = 0

        target_success = self.success_threshold if self.success_threshold > 0 else len(self.children)

        for child in self.children:
            if child.status != NodeStatus.SUCCESS and child.status != NodeStatus.FAILURE:
                status = child.tick()
            else:
                status = child.status

            if status == NodeStatus.SUCCESS:
                num_success += 1
            elif status == NodeStatus.FAILURE:
                num_failure += 1
            elif status == NodeStatus.RUNNING:
                num_running += 1

        if num_failure >= self.failure_threshold:
            return NodeStatus.FAILURE
        if num_success >= target_success:
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING
