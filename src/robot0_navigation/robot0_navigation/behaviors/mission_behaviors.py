import time
from typing import Optional, Union
from ..behavior_tree.node import ActionNode, NodeStatus, Blackboard
from ..arena_coordinates import (
    PALLETS,
    STORAGE_RACKS,
    DROPOFF_ZONES,
    LIFT_HEIGHT_TRANSIT,
    LIFT_HEIGHT_LEVEL1_INSERT,
    LIFT_HEIGHT_LEVEL1_CARRY,
    LIFT_HEIGHT_LEVEL2_INSERT,
    LIFT_HEIGHT_LEVEL2_CARRY,
    LIFT_HEIGHT_DROPOFF,
    LIFT_HEIGHT_TOLERANCE,
    LIFT_TIMEOUT_SEC,
    find_pallet_by_type,
    find_pallet_by_rack_and_slot,
    get_default_dropoff_for_pallet,
    calculate_pallet_pick_poses,
    generate_approach_route,
    generate_delivery_route,
    generate_return_home_route,
)


class InitializeMissionAction(ActionNode):
    """
    Action Node: Khởi tạo thông số nhiệm vụ lên Blackboard.
    Phân tích tham số, tra cứu Pallet, DropOffZone, tính toán các mốc độ cao
    và tọa độ di chuyển cần thiết cho toàn bộ cây hành vi.
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
        Thực thi mỗi chu kỳ tick: Phân tích tham số nhiệm vụ và thiết lập lên Blackboard.
        """
        ros_node = self.blackboard.get('ros_node')

        # 1. Đọc tham số từ Blackboard nếu có (ưu tiên tham số truyền vào launch), hoặc dùng default
        pallet_type_param = self.blackboard.get('param_pallet_type', self.pallet_type)
        target_rack_param = self.blackboard.get('param_target_rack', self.target_rack)
        shelf_level_param = int(self.blackboard.get('param_shelf_level', self.shelf_level))
        target_slot_param = self.blackboard.get('param_target_slot', self.target_slot)
        dropoff_zone_param = self.blackboard.get('param_dropoff_zone', self.dropoff_zone)

        # 2. Xác định Pallet mục tiêu
        pallet = None
        if pallet_type_param:
            pallet = find_pallet_by_type(pallet_type_param)
        if pallet is None:
            pallet = find_pallet_by_rack_and_slot(target_rack_param, shelf_level_param, target_slot_param)
        if pallet is None:
            pallet = PALLETS['pallet_aluminum']
            if ros_node:
                ros_node.get_logger().warn(
                    f"[BT] Could not resolve pallet ({pallet_type_param} / {target_rack_param}-{shelf_level_param}-{target_slot_param}). "
                    f"Defaulting to '{pallet.name}'."
                )

        # 3. Xác định Vùng giao hàng (Drop-off Zone)
        dropoff_zone = None
        if dropoff_zone_param and dropoff_zone_param in DROPOFF_ZONES:
            dropoff_zone = DROPOFF_ZONES[dropoff_zone_param]
        else:
            dropoff_zone = get_default_dropoff_for_pallet(pallet)

        # 4. Tính toán bộ 3 tọa độ tiếp cận / xỏ càng / lùi rút
        staging_pose, insert_pose, retract_pose = calculate_pallet_pick_poses(pallet)

        # 5. Sinh các lộ trình di chuyển
        approach_route = generate_approach_route(pallet.rack)
        delivery_route = generate_delivery_route(pallet.rack, dropoff_zone)
        return_home_route = generate_return_home_route(dropoff_zone.approach_pose.y)

        # 6. Xác định độ cao càng nâng theo tầng kệ của Pallet
        lift_transit_height = LIFT_HEIGHT_TRANSIT
        if pallet.shelf == 'bottom':
            lift_insert_height = LIFT_HEIGHT_LEVEL1_INSERT
            lift_carry_height = LIFT_HEIGHT_LEVEL1_CARRY
        else:
            lift_insert_height = LIFT_HEIGHT_LEVEL2_INSERT
            lift_carry_height = LIFT_HEIGHT_LEVEL2_CARRY
        lift_dropoff_height = LIFT_HEIGHT_DROPOFF

        # 7. Nạp toàn bộ thông số lên Blackboard
        rack_approach = STORAGE_RACKS[pallet.rack].approach_pose
        self.blackboard.set('target_pallet', pallet)
        self.blackboard.set('target_dropoff_zone', dropoff_zone)
        self.blackboard.set('rack_approach_pose', rack_approach)
        self.blackboard.set('staging_pose', staging_pose)
        self.blackboard.set('insert_pose', insert_pose)
        self.blackboard.set('retract_pose', retract_pose)
        self.blackboard.set('approach_route', approach_route)
        self.blackboard.set('delivery_route', delivery_route)
        self.blackboard.set('return_home_route', return_home_route)

        self.blackboard.set('lift_transit_height', lift_transit_height)
        self.blackboard.set('lift_insert_height', lift_insert_height)
        self.blackboard.set('lift_carry_height', lift_carry_height)
        self.blackboard.set('lift_dropoff_height', lift_dropoff_height)

        if ros_node:
            ros_node.get_logger().info(
                f"[BT] Mission Initialized: Target '{pallet.name}' (Rack: {pallet.rack}, Shelf: {pallet.shelf}, Slot: {pallet.slot}) "
                f"-> Destination '{dropoff_zone.name}'"
            )
            ros_node.get_logger().info(
                f"[BT] Lift Heights: Transit={lift_transit_height:.4f}m, Insert={lift_insert_height:.4f}m, "
                f"Carry={lift_carry_height:.4f}m, Dropoff={lift_dropoff_height:.4f}m"
            )

        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        """Được gọi khi node kết thúc (SUCCESS hoặc FAILURE)."""
        pass


class SetLiftHeightAction(ActionNode):
    """
    Action Node: Điều khiển vòng kín cơ cấu nâng hạ càng robot (/lift_joint_cmd).
    Theo dõi phản hồi vị trí thực tế (/joint_states) qua Blackboard, kiểm tra dung sai và thời gian ổn định.
    """
    def __init__(
        self,
        name: str,
        target_height: Union[float, str],
        tolerance: float = LIFT_HEIGHT_TOLERANCE,
        settle_time_sec: float = 1.0,
        timeout_sec: float = LIFT_TIMEOUT_SEC,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.target_height_spec = target_height
        self.tolerance = tolerance
        self.settle_time_sec = settle_time_sec
        self.timeout_sec = timeout_sec

        self.target_height: float = 0.0
        self.start_time: float = 0.0
        self.settle_start_time: Optional[float] = None

    def initialise(self) -> None:
        """Được gọi khi node bắt đầu chuyển sang trạng thái thực thi."""
        if isinstance(self.target_height_spec, str):
            val = self.blackboard.get(self.target_height_spec)
            if val is not None:
                self.target_height = float(val)
            else:
                ros_node = self.blackboard.get('ros_node')
                if ros_node:
                    ros_node.get_logger().warn(
                        f"[BT] SetLiftHeightAction '{self.name}': Key '{self.target_height_spec}' not found in Blackboard! Defaulting to 0.0m."
                    )
                self.target_height = 0.0
        else:
            self.target_height = float(self.target_height_spec)

        self.start_time = time.time()
        self.settle_start_time = None

        # Gửi lệnh nâng tới ROS node
        ros_node = self.blackboard.get('ros_node')
        if ros_node and hasattr(ros_node, 'set_lift'):
            ros_node.set_lift(self.target_height)
            ros_node.get_logger().info(
                f"[BT] SetLiftHeightAction '{self.name}': Commanding target height = {self.target_height:.4f} m"
            )

    def update(self) -> NodeStatus:
        """
        Thực thi mỗi chu kỳ tick:
        Kiểm tra phản hồi vị trí thực tế của khớp lift_arm_joint so với mục tiêu.
        """
        now = time.time()
        ros_node = self.blackboard.get('ros_node')

        # 1. Kiểm tra quá thời gian thực thi (Timeout)
        if now - self.start_time > self.timeout_sec:
            if ros_node:
                ros_node.get_logger().error(
                    f"[BT] SetLiftHeightAction '{self.name}': TIMEOUT ({self.timeout_sec}s) reaching {self.target_height:.4f}m!"
                )
            return NodeStatus.FAILURE

        # 2. Đọc vị trí thực tế từ Blackboard
        actual_pos = self.blackboard.get('actual_lift_pos')
        if actual_pos is None:
            # Chưa nhận được JointState từ Gazebo
            return NodeStatus.RUNNING

        # 3. Kiểm tra sai số
        error = abs(float(actual_pos) - self.target_height)

        if error <= self.tolerance:
            if self.settle_start_time is None:
                self.settle_start_time = now

            if now - self.settle_start_time >= self.settle_time_sec:
                if ros_node:
                    ros_node.get_logger().info(
                        f"[BT] SetLiftHeightAction '{self.name}': SUCCESS reached target {self.target_height:.4f}m "
                        f"(actual: {actual_pos:.4f}m, err: {error*1000.0:.1f}mm)"
                    )
                return NodeStatus.SUCCESS
        else:
            # Đang di chuyển hoặc dao động ra ngoài dung sai
            self.settle_start_time = None

        return NodeStatus.RUNNING

    def terminate(self, new_status: NodeStatus) -> None:
        """Được gọi khi node kết thúc (SUCCESS hoặc FAILURE)."""
        pass

