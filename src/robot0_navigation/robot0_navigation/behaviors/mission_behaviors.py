# -*- coding: utf-8 -*-

"""
Mission Behavior Action Nodes (Architectural Skeleton).
Provides high-level mission actions for initialization and lift manipulation.
Execution logic has been stripped to clean skeleton templates with TODO placeholders.
"""

from typing import Optional, Union
from ..behavior_tree.node import ActionNode, NodeStatus, Blackboard


class InitializeMissionAction(ActionNode):
    """
    Skeleton Action Node: Khởi tạo thông số nhiệm vụ lên Blackboard.
    (Ví dụ: Cấu hình kệ mục tiêu, tầng pallet, vùng trả hàng).
    """
    def __init__(
        self,
        name: str,
        target_rack: str = 'rack_1',
        shelf_level: int = 1,
        target_slot: str = 'left',
        pallet_type: str = '',
        dropoff_zone: str = '',
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.target_rack = target_rack
        self.shelf_level = shelf_level
        self.target_slot = target_slot
        self.pallet_type = pallet_type
        self.dropoff_zone = dropoff_zone

    def initialise(self) -> None:
        """Được gọi khi node bắt đầu thực thi."""
        pass

    def update(self) -> NodeStatus:
        """
        Thực thi mỗi chu kỳ tick:
        TODO: Cài đặt logic phân tích tham số nhiệm vụ và thiết lập các biến mục tiêu lên Blackboard.
        """
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.get_logger().info(f"[BT] Initializing mission configuration (Skeleton)...")

        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        """Được gọi khi node kết thúc (SUCCESS hoặc FAILURE)."""
        pass


class SetLiftHeightAction(ActionNode):
    """
    Skeleton Action Node: Điều khiển cơ cấu nâng hạ càng robot đến độ cao mục tiêu.
    """
    def __init__(
        self,
        name: str,
        target_height: Union[float, str],
        settle_time_sec: float = 1.0,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.target_height_spec = target_height
        self.settle_time_sec = settle_time_sec

    def initialise(self) -> None:
        """Được gọi khi node bắt đầu chuyển sang trạng thái thực thi."""
        pass

    def update(self) -> NodeStatus:
        """
        Thực thi mỗi chu kỳ tick:
        TODO: Gửi lệnh độ cao nâng (/lift_joint_cmd), kiểm tra phản hồi cảm biến (/joint_states),
        và trả về RUNNING trong khi đang chuyển động, SUCCESS khi đã đạt độ cao.
        """
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.get_logger().info(f"[BT] SetLiftHeightAction '{self.name}' (Skeleton reached target).")

        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        """Được gọi khi node kết thúc hoặc bị hủy bỏ."""
        pass

