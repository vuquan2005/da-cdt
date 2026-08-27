# -*- coding: utf-8 -*-

import time
from enum import Enum
from typing import Any, Dict, Optional


class NodeStatus(Enum):
    INVALID = 'INVALID'
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


class Blackboard:
    """Thread-safe shared key-value storage across the entire Behavior Tree."""
    def __init__(self):
        self._data: Dict[str, Any] = {}

    def set(self, key: str, value: Any) -> None:
        self._data[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)

    def has(self, key: str) -> bool:
        return key in self._data

    def clear(self) -> None:
        self._data.clear()

    def snapshot(self) -> Dict[str, Any]:
        return dict(self._data)


class TreeNode:
    """Base class for all Behavior Tree Nodes."""
    def __init__(self, name: str, blackboard: Optional[Blackboard] = None):
        self.name: str = name
        self.status: NodeStatus = NodeStatus.INVALID
        self.blackboard: Blackboard = blackboard if blackboard is not None else Blackboard()
        self.parent: Optional['TreeNode'] = None
        self._last_tick_time: float = 0.0

    def tick(self) -> NodeStatus:
        """Executes node lifecycle: initialise -> update -> terminate."""
        if self.status != NodeStatus.RUNNING:
            self.initialise()

        self._last_tick_time = time.time()
        self.status = self.update()

        if self.status != NodeStatus.RUNNING:
            self.terminate(self.status)

        return self.status

    def initialise(self) -> None:
        """Called when the node transitions from non-RUNNING to RUNNING."""
        pass

    def update(self) -> NodeStatus:
        """Main execution body called on every tick while RUNNING."""
        raise NotImplementedError('Subclasses must implement update()')

    def terminate(self, new_status: NodeStatus) -> None:
        """Called when the node finishes execution (SUCCESS or FAILURE)."""
        pass

    def reset(self) -> None:
        """Resets the node state back to INVALID."""
        self.status = NodeStatus.INVALID

    def attach_blackboard(self, blackboard: Blackboard) -> None:
        self.blackboard = blackboard


class ActionNode(TreeNode):
    """Leaf node that performs a concrete action (actuators, publishers, calculations)."""
    pass


class ConditionNode(TreeNode):
    """Leaf node that checks a condition and immediately returns SUCCESS or FAILURE."""
    def update(self) -> NodeStatus:
        if self.check():
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    def check(self) -> bool:
        raise NotImplementedError('Subclasses must implement check()')
