#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Arena Coordinates and Waypoint Definition for Robot0 Simulation.
Single Source of Truth for:
- Robot Spawn & Start Zone
- Storage Racks (Rack 1 & Rack 2)
- Pallets (Aluminum, CPU, QR, Chip) across Bottom & Top Shelves
- 4 Central Drop-Off Zones
- Kinematic offsets and Lift Height constants
"""

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass(frozen=True)
class StorageRack:
    name: str
    description: str
    pose: Pose3D
    approach_pose: Pose2D


@dataclass(frozen=True)
class Pallet:
    name: str
    rack: str
    shelf: str        # 'bottom' (level 1) or 'top' (level 2)
    slot: str         # 'left' or 'right'
    item_type: str    # 'aluminum', 'cpu', 'qr', 'chip'
    block_id: int
    pose: Pose3D


@dataclass(frozen=True)
class DropOffZone:
    name: str
    index: int
    item_type: str
    description: str
    center_pose: Pose3D
    approach_pose: Pose2D


# ==============================================================================
# 1. ROBOT SPAWN POSE & CONSTANTS
# ==============================================================================
ROBOT_SPAWN = Pose3D(x=-0.985, y=0.640, z=0.080, yaw=math.pi)

# Kinematic offsets
LIFT_ARM_LATERAL_OFFSET = 0.00827  # 8.27mm offset of lift_arm_joint in base_link (moves robot Y)
FORK_REACH_DISTANCE = 0.2308       # 23.08cm distance from robot center to fork tips

# Lift Height settings (meters)
LIFT_HEIGHT_TRANSIT = 0.015        # Safe ground clearance while driving
LIFT_HEIGHT_LEVEL1_INSERT = 0.0295 # Align fork with bottom shelf pallet opening
LIFT_HEIGHT_LEVEL1_CARRY = 0.0700  # Elevated above bottom shelf
LIFT_HEIGHT_LEVEL2_INSERT = 0.1495 # Align fork with middle/top shelf pallet opening
LIFT_HEIGHT_LEVEL2_CARRY = 0.1850  # Elevated above middle/top shelf
LIFT_HEIGHT_DROPOFF = 0.0000       # Lowered completely for pallet release


# ==============================================================================
# 2. STORAGE RACKS (2 Racks in Simplified Arena)
# ==============================================================================
STORAGE_RACKS: Dict[str, StorageRack] = {
    'rack_1': StorageRack(
        name='rack_1',
        description='Kệ 1 (Hàng dưới, Y = 0.640m)',
        pose=Pose3D(x=-1.894, y=0.640, z=0.0025, yaw=1.5707963),
        approach_pose=Pose2D(x=-1.500, y=0.640, yaw=math.pi),
    ),
    'rack_2': StorageRack(
        name='rack_2',
        description='Kệ 2 (Hàng giữa, Y = 0.000m)',
        pose=Pose3D(x=-1.894, y=0.000, z=0.0025, yaw=1.5707963),
        approach_pose=Pose2D(x=-1.500, y=0.000, yaw=math.pi),
    ),
}


# ==============================================================================
# 3. PALLETS (4 Pallets across 2 Racks)
# ==============================================================================
PALLETS: Dict[str, Pallet] = {
    # --- RACK 1: Aluminum (Bottom-Left), CPU (Top-Right) ---
    'pallet_aluminum': Pallet(
        name='pallet_aluminum',
        rack='rack_1', shelf='bottom', slot='left', item_type='aluminum', block_id=0,
        pose=Pose3D(x=-1.894, y=0.580, z=0.0285, yaw=1.5708)
    ),
    'pallet_cpu': Pallet(
        name='pallet_cpu',
        rack='rack_1', shelf='top', slot='right', item_type='cpu', block_id=1,
        pose=Pose3D(x=-1.894, y=0.700, z=0.1485, yaw=1.5708)
    ),

    # --- RACK 2: QR Code (Bottom-Left), Chip (Top-Right) ---
    'pallet_qr': Pallet(
        name='pallet_qr',
        rack='rack_2', shelf='bottom', slot='left', item_type='qr', block_id=2,
        pose=Pose3D(x=-1.894, y=-0.060, z=0.0285, yaw=1.5708)
    ),
    'pallet_chip': Pallet(
        name='pallet_chip',
        rack='rack_2', shelf='top', slot='right', item_type='chip', block_id=3,
        pose=Pose3D(x=-1.894, y=0.060, z=0.1485, yaw=1.5708)
    ),
}


# ==============================================================================
# 4. 4 DROP-OFF ZONES (East side, X = 0.70m)
# ==============================================================================
DROPOFF_ZONES: Dict[str, DropOffZone] = {
    'dropoff_1': DropOffZone(
        name='dropoff_1', index=1, item_type='aluminum',
        description='Vùng 1: Trả Pallet Nhôm (Xanh lam, Y = 0.64m)',
        center_pose=Pose3D(x=0.70, y=0.64, z=0.0, yaw=0.0),
        approach_pose=Pose2D(x=0.55, y=0.64, yaw=0.0),
    ),
    'dropoff_2': DropOffZone(
        name='dropoff_2', index=2, item_type='cpu',
        description='Vùng 2: Trả Pallet CPU (Xanh lá, Y = 0.22m)',
        center_pose=Pose3D(x=0.70, y=0.22, z=0.0, yaw=0.0),
        approach_pose=Pose2D(x=0.55, y=0.22, yaw=0.0),
    ),
    'dropoff_3': DropOffZone(
        name='dropoff_3', index=3, item_type='qr',
        description='Vùng 3: Trả Pallet QR Code (Vàng, Y = -0.22m)',
        center_pose=Pose3D(x=0.70, y=-0.22, z=0.0, yaw=0.0),
        approach_pose=Pose2D(x=0.55, y=-0.22, yaw=0.0),
    ),
    'dropoff_4': DropOffZone(
        name='dropoff_4', index=4, item_type='chip',
        description='Vùng 4: Trả Pallet Chip (Đỏ, Y = -0.64m)',
        center_pose=Pose3D(x=0.70, y=-0.64, z=0.0, yaw=0.0),
        approach_pose=Pose2D(x=0.55, y=-0.64, yaw=0.0),
    ),
}


# ==============================================================================
# 5. LINE GRID INTERSECTIONS (Topological Graph for Simulated Line Following)
# ==============================================================================
@dataclass(frozen=True)
class LineIntersection:
    name: str
    description: str
    pose: Pose2D


LINE_INTERSECTIONS: Dict[str, LineIntersection] = {
    # West approach points (X = -1.500m, in front of storage racks)
    'I_WEST_RACK1': LineIntersection('I_WEST_RACK1', 'Vị trí tiếp cận trước Kệ 1', Pose2D(x=-1.500, y=0.640, yaw=math.pi)),
    'I_WEST_RACK2': LineIntersection('I_WEST_RACK2', 'Vị trí tiếp cận trước Kệ 2', Pose2D(x=-1.500, y=0.000, yaw=math.pi)),

    # Start column (X = -0.985m)
    'I_START': LineIntersection('I_START', 'Vị trí trạm xuất phát ban đầu', Pose2D(x=-0.985, y=0.640, yaw=math.pi)),

    # Switch column (X = -0.400m, lane transition)
    'I_SWITCH_TOP': LineIntersection('I_SWITCH_TOP', 'Ngã 3 chuyển làn trên (Y=0.64m)', Pose2D(x=-0.400, y=0.640, yaw=0.0)),
    'I_SWITCH_BOT': LineIntersection('I_SWITCH_BOT', 'Ngã 3 chuyển làn dưới (Y=0.00m)', Pose2D(x=-0.400, y=0.000, yaw=0.0)),

    # Center vertical trunk line (X = 0.000m, central distribution axis)
    'I_CENTER_NORTH': LineIntersection('I_CENTER_NORTH', 'Giao lộ trục giữa - Bắc (Y=0.64m)', Pose2D(x=0.000, y=0.640, yaw=0.0)),
    'I_CENTER_MID_N': LineIntersection('I_CENTER_MID_N', 'Giao lộ trục giữa - Dropoff 2 (Y=0.22m)', Pose2D(x=0.000, y=0.220, yaw=0.0)),
    'I_CENTER_MID': LineIntersection('I_CENTER_MID', 'Giao lộ trục giữa - Ngang Kệ 2 (Y=0.00m)', Pose2D(x=0.000, y=0.000, yaw=0.0)),
    'I_CENTER_MID_S': LineIntersection('I_CENTER_MID_S', 'Giao lộ trục giữa - Dropoff 3 (Y=-0.22m)', Pose2D(x=0.000, y=-0.220, yaw=0.0)),
    'I_CENTER_SOUTH': LineIntersection('I_CENTER_SOUTH', 'Giao lộ trục giữa - Nam (Y=-0.64m)', Pose2D(x=0.000, y=-0.640, yaw=0.0)),

    # East vertical trunk line (X = 0.550m, in front of Dropoff Zones)
    'I_EAST_DROP1': LineIntersection('I_EAST_DROP1', 'Giao lộ tiếp cận Vùng 1 (Nhôm, Y=0.64m)', Pose2D(x=0.550, y=0.640, yaw=0.0)),
    'I_EAST_DROP2': LineIntersection('I_EAST_DROP2', 'Giao lộ tiếp cận Vùng 2 (CPU, Y=0.22m)', Pose2D(x=0.550, y=0.220, yaw=0.0)),
    'I_EAST_DROP3': LineIntersection('I_EAST_DROP3', 'Giao lộ tiếp cận Vùng 3 (QR, Y=-0.22m)', Pose2D(x=0.550, y=-0.220, yaw=0.0)),
    'I_EAST_DROP4': LineIntersection('I_EAST_DROP4', 'Giao lộ tiếp cận Vùng 4 (Chip, Y=-0.64m)', Pose2D(x=0.550, y=-0.640, yaw=0.0)),
}


# ==============================================================================
# 6. HELPER QUERY & ROUTE GENERATION FUNCTIONS
# ==============================================================================
def find_pallet_by_type(item_type: str) -> Optional[Pallet]:
    item_type = item_type.lower().strip()
    for pallet in PALLETS.values():
        if pallet.item_type == item_type or item_type in pallet.name:
            return pallet
    return None


def find_pallet_by_rack_and_slot(rack_name: str, shelf_level: int, slot: str) -> Optional[Pallet]:
    shelf_name = 'bottom' if shelf_level == 1 else 'top'
    slot_name = slot.lower().strip()
    for pallet in PALLETS.values():
        if pallet.rack == rack_name and pallet.shelf == shelf_name and pallet.slot == slot_name:
            return pallet
    return None


def get_default_dropoff_for_pallet(pallet: Pallet) -> DropOffZone:
    for zone in DROPOFF_ZONES.values():
        if zone.item_type == pallet.item_type:
            return zone
    return DROPOFF_ZONES['dropoff_1']


def generate_approach_route(target_rack: str, staging_pose: Pose2D) -> list:
    """
    Generates simulated line-following waypoint route from Home Spawn to Rack Staging Pose.
    Explicitly visits all intermediate line intersections.
    """
    route = []
    if target_rack == 'rack_2':
        # Start (X=-0.985, Y=0.64) -> Switch Top (X=-0.400, Y=0.64) -> Switch Bot (X=-0.400, Y=0.00) -> Rack 2 Approach (X=-1.500, Y=0.00)
        route.append(LINE_INTERSECTIONS['I_SWITCH_TOP'].pose)
        route.append(LINE_INTERSECTIONS['I_SWITCH_BOT'].pose)
        route.append(LINE_INTERSECTIONS['I_WEST_RACK2'].pose)
    else:
        # Start (X=-0.985, Y=0.64) -> Rack 1 Approach (X=-1.500, Y=0.64)
        route.append(LINE_INTERSECTIONS['I_WEST_RACK1'].pose)

    # Final staging alignment in front of pallet slot
    route.append(staging_pose)
    return route


def generate_delivery_route(rack_name: str, staging_pose: Pose2D, dropoff_zone: Optional[DropOffZone]) -> list:
    """
    Generates simulated line-following waypoint route from Rack Retract Pose to Drop-off Zone.
    Safely backs out to X = -1.350m before rotating, avoiding any collision with storage racks.
    """
    route = []
    target_y = dropoff_zone.approach_pose.y if dropoff_zone else 0.640

    if rack_name == 'rack_1':
        # 1. Back out cleanly to safe open track line (X=-1.350m, Y=0.640m) facing East
        route.append(Pose2D(x=-1.350, y=0.640, yaw=0.0))
        # 2. Pass through Start Intersection
        route.append(Pose2D(x=-0.985, y=0.640, yaw=0.0))
        # 3. Pass through Switch Top Intersection
        route.append(Pose2D(x=-0.400, y=0.640, yaw=0.0))
        # 4. Arrive at Central Distribution Top
        route.append(Pose2D(x=0.000, y=0.640, yaw=0.0))

        # 5. Travel down Central Distribution Column if needed
        if abs(target_y - 0.220) < 0.05:
            route.append(Pose2D(x=0.000, y=0.220, yaw=0.0))
        elif abs(target_y - (-0.220)) < 0.05:
            route.append(Pose2D(x=0.000, y=0.220, yaw=0.0))
            route.append(Pose2D(x=0.000, y=0.000, yaw=0.0))
            route.append(Pose2D(x=0.000, y=-0.220, yaw=0.0))
        elif abs(target_y - (-0.640)) < 0.05:
            route.append(Pose2D(x=0.000, y=0.220, yaw=0.0))
            route.append(Pose2D(x=0.000, y=0.000, yaw=0.0))
            route.append(Pose2D(x=0.000, y=-0.220, yaw=0.0))
            route.append(Pose2D(x=0.000, y=-0.640, yaw=0.0))

    else:  # rack_2 (Y = 0.000m)
        # 1. Back out cleanly to safe open track line (X=-1.350m, Y=0.000m) facing East
        route.append(Pose2D(x=-1.350, y=0.000, yaw=0.0))
        # 2. Pass through Switch Bot Intersection
        route.append(Pose2D(x=-0.400, y=0.000, yaw=0.0))
        # 3. Arrive at Central Distribution Mid
        route.append(Pose2D(x=0.000, y=0.000, yaw=0.0))

        # 4. Travel up/down Central Distribution Column
        if abs(target_y - 0.640) < 0.05:
            route.append(Pose2D(x=0.000, y=0.220, yaw=0.0))
            route.append(Pose2D(x=0.000, y=0.640, yaw=0.0))
        elif abs(target_y - 0.220) < 0.05:
            route.append(Pose2D(x=0.000, y=0.220, yaw=0.0))
        elif abs(target_y - (-0.220)) < 0.05:
            route.append(Pose2D(x=0.000, y=-0.220, yaw=0.0))
        elif abs(target_y - (-0.640)) < 0.05:
            route.append(Pose2D(x=0.000, y=-0.220, yaw=0.0))
            route.append(Pose2D(x=0.000, y=-0.640, yaw=0.0))

    # 6. Branch out East to target Drop-off Zone
    if dropoff_zone:
        route.append(dropoff_zone.approach_pose)
    else:
        route.append(Pose2D(x=0.550, y=0.640, yaw=0.0))

    return route


def generate_return_home_route(current_dropoff_y: float) -> list:
    """
    Generates simulated line-following waypoint route from Drop-off area back to Home Base.
    Explicitly visits all intermediate line intersections.
    """
    route = []
    # 1. Back out from Drop-off to Central Column
    route.append(Pose2D(x=0.000, y=current_dropoff_y, yaw=math.pi))

    # 2. Travel up Central Column to North Intersection (Y=0.640m)
    if abs(current_dropoff_y - (-0.640)) < 0.05:
        route.append(Pose2D(x=0.000, y=-0.220, yaw=math.pi))
        route.append(Pose2D(x=0.000, y=0.000, yaw=math.pi))
        route.append(Pose2D(x=0.000, y=0.220, yaw=math.pi))
        route.append(Pose2D(x=0.000, y=0.640, yaw=math.pi))
    elif abs(current_dropoff_y - (-0.220)) < 0.05:
        route.append(Pose2D(x=0.000, y=0.000, yaw=math.pi))
        route.append(Pose2D(x=0.000, y=0.220, yaw=math.pi))
        route.append(Pose2D(x=0.000, y=0.640, yaw=math.pi))
    elif abs(current_dropoff_y - 0.220) < 0.05:
        route.append(Pose2D(x=0.000, y=0.640, yaw=math.pi))

    # 3. Travel West along Row 1 through Switch Top back to Start
    route.append(Pose2D(x=-0.400, y=0.640, yaw=math.pi))
    route.append(Pose2D(x=ROBOT_SPAWN.x, y=ROBOT_SPAWN.y, yaw=ROBOT_SPAWN.yaw))
    return route
