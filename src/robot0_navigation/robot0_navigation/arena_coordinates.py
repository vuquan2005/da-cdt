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
        description='Kệ 1 (Hàng dưới, Y ≈ 0.641m)',
        pose=Pose3D(x=-1.894, y=0.641, z=0.0025, yaw=1.5707963),
        approach_pose=Pose2D(x=-1.600, y=0.641, yaw=math.pi),
    ),
    'rack_2': StorageRack(
        name='rack_2',
        description='Kệ 2 (Hàng giữa, Y ≈ -0.006m)',
        pose=Pose3D(x=-1.894, y=-0.006, z=0.0025, yaw=1.5707963),
        approach_pose=Pose2D(x=-1.600, y=-0.006, yaw=math.pi),
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
        pose=Pose3D(x=-1.894, y=0.581, z=0.0285, yaw=1.5708)
    ),
    'pallet_cpu': Pallet(
        name='pallet_cpu',
        rack='rack_1', shelf='top', slot='right', item_type='cpu', block_id=1,
        pose=Pose3D(x=-1.894, y=0.701, z=0.1485, yaw=1.5708)
    ),

    # --- RACK 2: QR Code (Bottom-Left), Chip (Top-Right) ---
    'pallet_qr': Pallet(
        name='pallet_qr',
        rack='rack_2', shelf='bottom', slot='left', item_type='qr', block_id=2,
        pose=Pose3D(x=-1.894, y=-0.066, z=0.0285, yaw=1.5708)
    ),
    'pallet_chip': Pallet(
        name='pallet_chip',
        rack='rack_2', shelf='top', slot='right', item_type='chip', block_id=3,
        pose=Pose3D(x=-1.894, y=0.054, z=0.1485, yaw=1.5708)
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
# 5. HELPER QUERY FUNCTIONS
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
