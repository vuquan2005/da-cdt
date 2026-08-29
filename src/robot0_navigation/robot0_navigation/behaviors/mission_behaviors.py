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


CLASS_MAP = {
    'al': 'aluminum',
    'aluminum': 'aluminum',
    'samsung': 'cpu',
    'cpu': 'cpu',
    'qr': 'qr',
    'chip': 'chip',
}

DROPOFF_BY_ITEM = {
    'aluminum': 'dropoff_1',
    'cpu': 'dropoff_2',
    'qr': 'dropoff_3',
    'chip': 'dropoff_4',
}


class ScanRackPalletsWithYoloAction(ActionNode):
    """
    Action Node: Quét và phân loại pallet trên kệ bằng YOLO khi robot đỗ trước kệ (X = -1.500m).
    Phân loại vị trí 4 ô khay:
      - Tầng (Shelf): 'top' (nếu Cy < 240) hoặc 'bottom' (nếu Cy >= 240)
      - Ngăn (Slot): 'left' (nếu Cx < 320) hoặc 'right' (nếu Cx >= 320)
    Khớp với loại hàng mục tiêu và tự động cập nhật:
      1. Tọa độ dạt khay (staging_pose, insert_pose, retract_pose)
      2. Độ cao nâng hạ (lift_insert_height, lift_carry_height)
      3. Vùng giao hàng chính xác (target_dropoff_zone, delivery_route, return_home_route)
    """
    def __init__(
        self,
        name: str,
        scan_duration_sec: float = 1.2,
        timeout_sec: float = 5.0,
        img_w: int = 640,
        img_h: int = 480,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.scan_duration_sec = scan_duration_sec
        self.timeout_sec = timeout_sec
        self.img_w = img_w
        self.img_h = img_h
        self.cx_threshold = img_w / 2.0
        self.cy_threshold = img_h / 2.0

        self.start_time: float = 0.0
        self.first_detection_time: Optional[float] = None
        self.accumulated_detections: list = []

    def initialise(self) -> None:
        self.start_time = time.time()
        self.first_detection_time = None
        self.accumulated_detections = []
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.get_logger().info(
                f"[BT] ScanRackPalletsWithYoloAction '{self.name}': Bắt đầu quét pallet qua camera..."
            )

    def update(self) -> NodeStatus:
        now = time.time()
        ros_node = self.blackboard.get('ros_node')

        # Nếu use_yolo tắt, bỏ qua scan và dùng cấu hình mặc định
        use_yolo = self.blackboard.get('param_use_yolo', True)
        if not use_yolo:
            if ros_node:
                ros_node.get_logger().info(f"[BT] {self.name}: param_use_yolo is False -> Using pre-set mission coordinates.")
            return NodeStatus.SUCCESS

        # Đọc dữ liệu YOLO mới nhất từ Blackboard
        yolo_data = self.blackboard.get('latest_yolo_detections')
        if yolo_data and isinstance(yolo_data, dict):
            detections = yolo_data.get('detections', [])
            if detections:
                if self.first_detection_time is None:
                    self.first_detection_time = now
                self.accumulated_detections.extend(detections)

        # Kiểm tra nếu đã thu thập đủ dữ liệu quét qua thời gian scan_duration_sec
        if self.first_detection_time is not None and (now - self.first_detection_time >= self.scan_duration_sec):
            return self._process_detections()

        # Kiểm tra Timeout
        if now - self.start_time >= self.timeout_sec:
            if self.accumulated_detections:
                return self._process_detections()
            else:
                if ros_node:
                    ros_node.get_logger().warn(
                        f"[BT] {self.name}: Timeout ({self.timeout_sec}s) without YOLO detection. Falling back to default coordinates."
                    )
                return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def _process_detections(self) -> NodeStatus:
        ros_node = self.blackboard.get('ros_node')

        # Nhóm và bình chọn detection theo (shelf, slot)
        slot_detections = {}

        for det in self.accumulated_detections:
            raw_cls = det.get('class_name', '').lower()
            item_type = CLASS_MAP.get(raw_cls, raw_cls)
            conf = float(det.get('confidence', 0.5))
            center = det.get('center', [self.img_w / 2.0, self.img_h / 2.0])
            cx, cy = center[0], center[1]

            shelf = 'top' if cy < self.cy_threshold else 'bottom'
            slot = 'left' if cx < self.cx_threshold else 'right'
            key = (shelf, slot)

            if key not in slot_detections:
                slot_detections[key] = {}
            slot_detections[key][item_type] = slot_detections[key].get(item_type, 0.0) + conf

        # Xác định class có điểm cao nhất ở mỗi ô
        classified_rack = {}
        for key, class_scores in slot_detections.items():
            best_class = max(class_scores.items(), key=lambda x: x[1])[0]
            classified_rack[key] = best_class

        if ros_node:
            ros_node.get_logger().info(f"[BT] YOLO Scan Results on Rack: {classified_rack}")

        # Lấy thông số nhiệm vụ hiện tại
        current_pallet = self.blackboard.get('target_pallet')
        target_pallet_type = self.blackboard.get('param_pallet_type', '')
        target_rack = self.blackboard.get('param_target_rack', 'rack_1')
        if current_pallet:
            target_rack = current_pallet.rack
            if not target_pallet_type:
                target_pallet_type = current_pallet.item_type

        # Tìm ô chứa pallet mục tiêu
        chosen_key = None
        chosen_type = target_pallet_type

        # Ưu tiên tìm loại hàng được yêu cầu
        if target_pallet_type:
            target_norm = CLASS_MAP.get(target_pallet_type.lower(), target_pallet_type.lower())
            for key, cls in classified_rack.items():
                if cls == target_norm:
                    chosen_key = key
                    chosen_type = target_norm
                    break

        # Nếu không tìm thấy loại yêu cầu hoặc chế độ tự chọn, lấy ô có độ tin cậy cao đầu tiên
        if chosen_key is None and classified_rack:
            chosen_key = list(classified_rack.keys())[0]
            chosen_type = classified_rack[chosen_key]

        if chosen_key is not None:
            shelf, slot = chosen_key
            if ros_node:
                ros_node.get_logger().info(
                    f"[BT] >> YOLO Selected Target: Item='{chosen_type}' at Shelf='{shelf}', Slot='{slot}' on '{target_rack}'"
                )

            # Cập nhật tọa độ thế giới chính xác cho ô này
            if target_rack == 'rack_1':
                y_coord = 0.580 if slot == 'left' else 0.700
            else:
                y_coord = -0.060 if slot == 'left' else 0.060
            z_coord = 0.0285 if shelf == 'bottom' else 0.1485

            from ..arena_coordinates import Pallet, Pose3D
            updated_pallet = Pallet(
                name=f"pallet_{chosen_type}_{target_rack}_{shelf}_{slot}",
                rack=target_rack,
                shelf=shelf,
                slot=slot,
                item_type=chosen_type,
                block_id=0,
                pose=Pose3D(x=-1.894, y=y_coord, z=z_coord, yaw=1.5708)
            )

            # Tính lại các tư thế gắp và lộ trình
            staging_pose, insert_pose, retract_pose = calculate_pallet_pick_poses(updated_pallet)

            if shelf == 'bottom':
                lift_insert_height = LIFT_HEIGHT_LEVEL1_INSERT
                lift_carry_height = LIFT_HEIGHT_LEVEL1_CARRY
            else:
                lift_insert_height = LIFT_HEIGHT_LEVEL2_INSERT
                lift_carry_height = LIFT_HEIGHT_LEVEL2_CARRY

            dropoff_key = DROPOFF_BY_ITEM.get(chosen_type, 'dropoff_1')
            dropoff_zone = DROPOFF_ZONES.get(dropoff_key, DROPOFF_ZONES['dropoff_1'])
            delivery_route = generate_delivery_route(target_rack, dropoff_zone)
            return_home_route = generate_return_home_route(dropoff_zone.approach_pose.y)

            # Cập nhật Blackboard
            self.blackboard.set('target_pallet', updated_pallet)
            self.blackboard.set('target_dropoff_zone', dropoff_zone)
            self.blackboard.set('staging_pose', staging_pose)
            self.blackboard.set('insert_pose', insert_pose)
            self.blackboard.set('retract_pose', retract_pose)
            self.blackboard.set('lift_insert_height', lift_insert_height)
            self.blackboard.set('lift_carry_height', lift_carry_height)
            self.blackboard.set('delivery_route', delivery_route)
            self.blackboard.set('return_home_route', return_home_route)

            if ros_node:
                ros_node.get_logger().info(
                    f"[BT] >> Dynamic Blackboard Updated: Dropoff='{dropoff_zone.name}' (Y={dropoff_zone.center_pose.y:.2f}m), "
                    f"Pick Staging Y={staging_pose.y:.3f}m, Lift Height={lift_insert_height:.4f}m"
                )

        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        pass

