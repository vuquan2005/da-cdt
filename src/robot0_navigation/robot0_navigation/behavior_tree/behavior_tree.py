# -*- coding: utf-8 -*-

import time
from typing import Optional, List
from .node import TreeNode, NodeStatus, Blackboard
from .composites import CompositeNode
from .decorators import DecoratorNode


class BehaviorTree:
    """Behavior Tree Controller & Visualizer."""
    def __init__(self, root: TreeNode, blackboard: Optional[Blackboard] = None):
        self.root: TreeNode = root
        self.blackboard: Blackboard = blackboard if blackboard is not None else (root.blackboard if root else Blackboard())
        if self.root:
            self.root.attach_blackboard(self.blackboard)
        self.tick_count: int = 0
        self.start_time: float = time.time()

    def tick(self) -> NodeStatus:
        self.tick_count += 1
        return self.root.tick()

    def reset(self) -> None:
        self.tick_count = 0
        self.root.reset()

    def render_ascii_tree(self, use_color: bool = True) -> str:
        """Renders real-time ASCII visualization of the Behavior Tree with color-coded node statuses."""
        lines: List[str] = []
        self._format_node(self.root, prefix='', is_last=True, lines=lines, use_color=use_color)
        return '\n'.join(lines)

    def _format_node(self, node: TreeNode, prefix: str, is_last: bool, lines: List[str], use_color: bool) -> None:
        # Colors (ANSI)
        RESET = '\033[0m' if use_color else ''
        BOLD = '\033[1m' if use_color else ''
        GREEN = '\033[92m' if use_color else ''
        YELLOW = '\033[93m' if use_color else ''
        RED = '\033[91m' if use_color else ''
        CYAN = '\033[96m' if use_color else ''
        GRAY = '\033[90m' if use_color else ''

        if node.status == NodeStatus.SUCCESS:
            status_tag = f'{GREEN}[SUCCESS]{RESET}'
        elif node.status == NodeStatus.RUNNING:
            status_tag = f'{YELLOW}[RUNNING]{RESET}'
        elif node.status == NodeStatus.FAILURE:
            status_tag = f'{RED}[FAILURE]{RESET}'
        else:
            status_tag = f'{GRAY}[INVALID]{RESET}'

        node_type = node.__class__.__name__
        branch = '└── ' if is_last else '├── '
        lines.append(f'{prefix}{branch}{CYAN}{node_type}{RESET}: {BOLD}{node.name}{RESET} {status_tag}')

        child_prefix = prefix + ('    ' if is_last else '│   ')

        if isinstance(node, CompositeNode):
            for i, child in enumerate(node.children):
                last_child = (i == len(node.children) - 1)
                self._format_node(child, child_prefix, last_child, lines, use_color)
        elif isinstance(node, DecoratorNode):
            self._format_node(node.child, child_prefix, True, lines, use_color)
