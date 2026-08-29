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
    generate_rack_to_rack_route,
    generate_return_home_from_rack_route,
)


class InitializeMissionAction(ActionNode):
    """
    Action Node: Khởi tạo thông số nhiệm vụ tìm kiếm & gắp hàng lên Blackboard.
    Thiết lập các lộ trình tiếp cận giữa các kệ và lộ trình rút lui về Home.
    """
    def __init__(
        self,
        name: str,
        target_rack: str = 'rack_1',
        pallet_type: str = '',
        dropoff_zone: str = '',
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.target_rack = target_rack
        self.pallet_type = pallet_type
        self.dropoff_zone = dropoff_zone

    def initialise(self) -> None:
        """Được gọi khi node bắt đầu thực thi."""
        pass

    def update(self) -> NodeStatus:
        """
        Thực thi mỗi chu kỳ tick: Nạp các thông số tìm kiếm và đường đi lên Blackboard.
        """
        ros_node = self.blackboard.get('ros_node')

        # 1. Đọc tham số nhiệm vụ
        pallet_type_param = self.blackboard.get('param_pallet_type', self.pallet_type)
        target_rack_param = self.blackboard.get('param_target_rack', self.target_rack)
        dropoff_zone_param = self.blackboard.get('param_dropoff_zone', self.dropoff_zone)

        # 2. Sinh các lộ trình di chuyển tìm kiếm giữa các kệ
        approach_route_rack1 = generate_approach_route('rack_1')
        approach_route_rack2 = generate_approach_route('rack_2')
        route_rack1_to_rack2 = generate_rack_to_rack_route('rack_1', 'rack_2')
        route_rack2_to_rack1 = generate_rack_to_rack_route('rack_2', 'rack_1')
        route_rack1_to_home = generate_return_home_from_rack_route('rack_1')
        route_rack2_to_home = generate_return_home_from_rack_route('rack_2')

        # 3. Nạp thông số chung lên Blackboard
        self.blackboard.set('lift_transit_height', LIFT_HEIGHT_TRANSIT)
        self.blackboard.set('lift_dropoff_height', LIFT_HEIGHT_DROPOFF)
        self.blackboard.set('approach_route_rack1', approach_route_rack1)
        self.blackboard.set('approach_route_rack2', approach_route_rack2)
        self.blackboard.set('route_rack1_to_rack2', route_rack1_to_rack2)
        self.blackboard.set('route_rack2_to_rack1', route_rack2_to_rack1)
        self.blackboard.set('route_rack1_to_home', route_rack1_to_home)
        self.blackboard.set('route_rack2_to_home', route_rack2_to_home)

        self.blackboard.set('rack_1_approach_pose', STORAGE_RACKS['rack_1'].approach_pose)
        self.blackboard.set('rack_2_approach_pose', STORAGE_RACKS['rack_2'].approach_pose)
        self.blackboard.set('rack_approach_pose', STORAGE_RACKS['rack_1'].approach_pose)

        if ros_node:
            ros_node.get_logger().info(
                f"[BT] Mission Initialized for Dynamic Search: Target Item='{pallet_type_param or 'ANY'}' | "
                f"Starting Search at '{target_rack_param}'"
            )

        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        """Được gọi khi node kết thúc."""
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


DROPOFF_BY_ITEM = {
    'aluminum': 'dropoff_1',
    'cpu': 'dropoff_2',
    'qr': 'dropoff_3',
    'chip': 'dropoff_4',
}


class ScanRackPalletsWithYoloAction(ActionNode):
    """
    Action Node: Quét và nhận diện pallet tại kệ (current_rack) bằng YOLOv8.
    Nếu tìm thấy loại pallet mục tiêu (target_pallet_type):
      - Cập nhật Blackboard (target_pallet, staging_pose, insert_pose, retract_pose,
        lift_insert_height, lift_carry_height, target_dropoff_zone, delivery_route, return_home_route).
      - Trả về NodeStatus.SUCCESS.
    Nếu KHÔNG tìm thấy trên kệ này (hoặc timeout):
      - Trả về NodeStatus.FAILURE để Behavior Tree Selector chuyển sang tìm kiếm tại kệ tiếp theo!
    """
    def __init__(
        self,
        name: str,
        current_rack: str = 'rack_1',
        scan_duration_sec: float = 1.2,
        timeout_sec: float = 4.0,
        img_w: int = 640,
        img_h: int = 480,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.current_rack = current_rack
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
        # Clear any stale detection from previous rack
        self.blackboard.set('latest_yolo_detections', None)
        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            target_type = self.blackboard.get('param_pallet_type', '')
            ros_node.get_logger().info(
                f"[BT] ScanRackPalletsWithYoloAction '{self.name}': Bắt đầu quét pallet tại '{self.current_rack}' "
                f"cho mục tiêu '{target_type or 'ANY'}'..."
            )

    def update(self) -> NodeStatus:
        now = time.time()
        ros_node = self.blackboard.get('ros_node')

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
                        f"[BT] {self.name}: Timeout ({self.timeout_sec}s) without YOLO detection on '{self.current_rack}'."
                    )
                return NodeStatus.FAILURE

        return NodeStatus.RUNNING

    def _process_detections(self) -> NodeStatus:
        ros_node = self.blackboard.get('ros_node')

        # Nhóm và bình chọn detection theo (shelf, slot)
        slot_detections = {}

        for det in self.accumulated_detections:
            item_type = det.get('class_name', '').lower().strip()
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
            ros_node.get_logger().info(f"[BT] YOLO Scan Results on '{self.current_rack}': {classified_rack}")

        # Lấy loại hàng mục tiêu
        target_pallet_type = self.blackboard.get('param_pallet_type', '')

        chosen_key = None
        chosen_type = target_pallet_type

        # 1. Nếu có chỉ định loại hàng mục tiêu:
        if target_pallet_type:
            target_norm = target_pallet_type.lower().strip()
            for key, cls in classified_rack.items():
                if cls == target_norm:
                    chosen_key = key
                    chosen_type = target_norm
                    break

            if chosen_key is None:
                if ros_node:
                    ros_node.get_logger().warn(
                        f"[BT] ❌ Target '{target_norm}' NOT found on '{self.current_rack}'. "
                        f"Detected: {list(classified_rack.values())}."
                    )
                return NodeStatus.FAILURE

        # 2. Nếu chế độ tự động (ANY): lấy ô đầu tiên phát hiện được
        else:
            if classified_rack:
                chosen_key = list(classified_rack.keys())[0]
                chosen_type = classified_rack[chosen_key]
            else:
                if ros_node:
                    ros_node.get_logger().warn(f"[BT] ❌ No pallets detected on '{self.current_rack}'.")
                return NodeStatus.FAILURE

        # 3. Khi đã tìm thấy mục tiêu trên kệ này -> Cấu hình động toàn bộ thông số
        shelf, slot = chosen_key
        target_rack = self.current_rack
        if ros_node:
            ros_node.get_logger().info(
                f"[BT] ✅ FOUND Target '{chosen_type}' on '{target_rack}' at Shelf='{shelf}', Slot='{slot}'!"
            )

        # Tọa độ thế giới chính xác cho ô này
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
        rack_approach = STORAGE_RACKS[target_rack].approach_pose

        # Cập nhật Blackboard
        self.blackboard.set('target_pallet', updated_pallet)
        self.blackboard.set('target_dropoff_zone', dropoff_zone)
        self.blackboard.set('rack_approach_pose', rack_approach)
        self.blackboard.set('staging_pose', staging_pose)
        self.blackboard.set('insert_pose', insert_pose)
        self.blackboard.set('retract_pose', retract_pose)
        self.blackboard.set('lift_insert_height', lift_insert_height)
        self.blackboard.set('lift_carry_height', lift_carry_height)
        self.blackboard.set('delivery_route', delivery_route)
        self.blackboard.set('return_home_route', return_home_route)

        if ros_node:
            ros_node.get_logger().info(
                f"[BT] >> Dynamic Target Configured: Dropoff='{dropoff_zone.name}' (Y={dropoff_zone.center_pose.y:.2f}m), "
                f"Pick Staging Y={staging_pose.y:.3f}m, Lift Height={lift_insert_height:.4f}m"
            )

        return NodeStatus.SUCCESS

    def terminate(self, new_status: NodeStatus) -> None:
        pass

