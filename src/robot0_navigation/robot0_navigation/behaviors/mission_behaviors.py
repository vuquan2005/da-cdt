# -*- coding: utf-8 -*-

import time
import math
from typing import Optional, Union
from ..behavior_tree.node import ActionNode, NodeStatus, Blackboard
from ..arena_coordinates import (
    ROBOT_SPAWN,
    STORAGE_RACKS,
    PALLETS,
    DROPOFF_ZONES,
    Pose2D,
    Pose3D,
    LIFT_ARM_LATERAL_OFFSET,
    LIFT_HEIGHT_TRANSIT,
    LIFT_HEIGHT_LEVEL1_INSERT,
    LIFT_HEIGHT_LEVEL1_CARRY,
    LIFT_HEIGHT_LEVEL2_INSERT,
    LIFT_HEIGHT_LEVEL2_CARRY,
    LIFT_HEIGHT_DROPOFF,
    find_pallet_by_type,
    find_pallet_by_rack_and_slot,
    get_default_dropoff_for_pallet,
    generate_approach_route,
    generate_delivery_route,
    generate_return_home_route,
)


class InitializeMissionAction(ActionNode):
    """
    Initializes the Mission Blackboard:
    Resolves Target Rack, Shelf Level, Slot, Pallet Coordinates,
    computes Staging & Insert Poses, Drop-Off Pose, and Lift Heights.
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

    def update(self) -> NodeStatus:
        target_rack = self.blackboard.get('param_target_rack', self.target_rack)
        shelf_level = int(self.blackboard.get('param_shelf_level', self.shelf_level))
        target_slot = self.blackboard.get('param_target_slot', self.target_slot)
        pallet_type = self.blackboard.get('param_pallet_type', self.pallet_type)
        dropoff_zone_req = self.blackboard.get('param_dropoff_zone', self.dropoff_zone)

        # 1. Resolve Target Pallet
        pallet = None
        if pallet_type:
            pallet = find_pallet_by_type(pallet_type)
        if not pallet:
            pallet = find_pallet_by_rack_and_slot(target_rack, shelf_level, target_slot)

        if not pallet:
            ros_node = self.blackboard.get('ros_node')
            if ros_node:
                ros_node.get_logger().error(
                    f'Invalid mission config: rack={target_rack}, level={shelf_level}, slot={target_slot}, type={pallet_type}'
                )
            return NodeStatus.FAILURE

        # 2. Compute Staging and Insertion Poses
        # Pallet center Y - 8.27mm offset of lift arm
        y_align = pallet.pose.y - LIFT_ARM_LATERAL_OFFSET
        target_yaw = math.pi

        # Safe Staging Pose (before shelf) and Insert Pose (fork fully engaged)
        staging_pose = Pose2D(x=-1.550, y=y_align, yaw=target_yaw)
        insert_pose = Pose2D(x=-1.645, y=y_align, yaw=target_yaw)

        # 3. Lift Heights
        if pallet.shelf == 'bottom' or shelf_level == 1:
            lift_insert_height = LIFT_HEIGHT_LEVEL1_INSERT
            lift_carry_height = LIFT_HEIGHT_LEVEL1_CARRY
        else:
            lift_insert_height = LIFT_HEIGHT_LEVEL2_INSERT
            lift_carry_height = LIFT_HEIGHT_LEVEL2_CARRY

        # 4. Resolve Drop-off Target Pose
        if dropoff_zone_req and dropoff_zone_req in DROPOFF_ZONES:
            dropoff_zone = DROPOFF_ZONES[dropoff_zone_req]
        elif dropoff_zone_req == 'home':
            dropoff_zone = None
        else:
            dropoff_zone = get_default_dropoff_for_pallet(pallet)

        if dropoff_zone:
            dropoff_pose = dropoff_zone.approach_pose
            dropoff_desc = dropoff_zone.description
        else:
            dropoff_pose = Pose2D(x=ROBOT_SPAWN.x, y=ROBOT_SPAWN.y, yaw=ROBOT_SPAWN.yaw)
            dropoff_desc = 'Trạm xuất phát ban đầu (Home Base)'

        home_pose = Pose2D(x=ROBOT_SPAWN.x, y=ROBOT_SPAWN.y, yaw=ROBOT_SPAWN.yaw)

        # 5. Generate Topological Line Routes (Simulated Line Following Intersections)
        approach_route = generate_approach_route(pallet.rack, staging_pose)
        delivery_route = generate_delivery_route(pallet.rack, staging_pose, dropoff_zone)
        dropoff_y = dropoff_zone.approach_pose.y if dropoff_zone else ROBOT_SPAWN.y
        return_home_route = generate_return_home_route(dropoff_y)

        # 6. Populate Blackboard
        self.blackboard.set('pallet_target', pallet)
        self.blackboard.set('staging_pose', staging_pose)
        self.blackboard.set('insert_pose', insert_pose)
        self.blackboard.set('lift_insert_height', lift_insert_height)
        self.blackboard.set('lift_carry_height', lift_carry_height)
        self.blackboard.set('lift_transit_height', LIFT_HEIGHT_TRANSIT)
        self.blackboard.set('lift_dropoff_height', LIFT_HEIGHT_DROPOFF)
        self.blackboard.set('dropoff_pose', dropoff_pose)
        self.blackboard.set('dropoff_desc', dropoff_desc)
        self.blackboard.set('home_pose', home_pose)
        self.blackboard.set('approach_route', approach_route)
        self.blackboard.set('delivery_route', delivery_route)
        self.blackboard.set('return_home_route', return_home_route)

        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.get_logger().info('================ MISSION PLAN INITIALIZED ================')
            ros_node.get_logger().info(f'  Target Pallet  : {pallet.name} ({pallet.item_type.upper()}) on {pallet.rack} [{pallet.shelf.upper()} - {pallet.slot.upper()}]')
            ros_node.get_logger().info(f'  Pallet Pose    : X={pallet.pose.x:.3f}m, Y={pallet.pose.y:.3f}m, Z={pallet.pose.z:.3f}m')
            ros_node.get_logger().info(f'  Approach Line  : {len(approach_route)} intersections to Staging (X={staging_pose.x:.3f}m, Y={staging_pose.y:.3f}m)')
            ros_node.get_logger().info(f'  Delivery Line  : {len(delivery_route)} intersections to {dropoff_desc}')
            ros_node.get_logger().info(f'  Return Line    : {len(return_home_route)} intersections back to Home Base')
            ros_node.get_logger().info(f'  Insert Pose    : X={insert_pose.x:.3f}m, Y={insert_pose.y:.3f}m')
            ros_node.get_logger().info(f'  Lift Heights   : Insert={lift_insert_height*100:.2f}cm, Carry={lift_carry_height*100:.2f}cm')
            ros_node.get_logger().info('==========================================================')

        return NodeStatus.SUCCESS


class SetLiftHeightAction(ActionNode):
    """
    Controls the robot lift arm prismatic joint to reach a target height.
    Continuously updates robot target lift and waits until joint physically reaches the target height.
    """
    def __init__(
        self,
        name: str,
        target_height: Union[float, str],
        settle_time_sec: float = 2.5,
        tolerance: float = 0.008,
        blackboard: Optional[Blackboard] = None
    ):
        super().__init__(name, blackboard)
        self.target_height_spec = target_height
        self.settle_time_sec = settle_time_sec
        self.tolerance = tolerance
        self.target_height_val = 0.0
        self.start_time = 0.0

    def initialise(self) -> None:
        self.start_time = time.time()
        if isinstance(self.target_height_spec, str):
            self.target_height_val = float(self.blackboard.get(self.target_height_spec, 0.0))
        else:
            self.target_height_val = float(self.target_height_spec)

        ros_node = self.blackboard.get('ros_node')
        if ros_node:
            ros_node.set_lift(self.target_height_val)
            ros_node.get_logger().info(
                f"[BT] SetLiftHeightAction '{self.name}': Commanded lift to {self.target_height_val*100:.2f}cm"
            )

    def update(self) -> NodeStatus:
        elapsed = time.time() - self.start_time
        actual_lift = self.blackboard.get('actual_lift_pos', None)
        ros_node = self.blackboard.get('ros_node')

        # Keep commanding target lift
        if ros_node:
            ros_node.set_lift(self.target_height_val)

        if actual_lift is not None:
            if abs(actual_lift - self.target_height_val) <= self.tolerance:
                if elapsed >= 0.8:  # Ensure dynamic settling
                    if ros_node:
                        ros_node.get_logger().info(
                            f"[BT] Lift height confirmed at {actual_lift*100:.2f}cm (Target: {self.target_height_val*100:.2f}cm)"
                        )
                    return NodeStatus.SUCCESS

        # Fallback to duration timeout if joint feedback is delayed
        if elapsed >= self.settle_time_sec:
            if ros_node:
                ros_node.get_logger().info(
                    f"[BT] Lift settle time elapsed ({self.settle_time_sec:.1f}s) for {self.target_height_val*100:.2f}cm"
                )
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING
