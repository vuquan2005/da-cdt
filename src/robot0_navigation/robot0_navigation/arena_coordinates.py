"""
Arena Coordinates and Waypoint Definition for Robot0 Simulation.
Single Source of Truth for:
- Robot Spawn & Start Zone
- 4 Storage Racks
- 16 Pallets (Aluminum, CPU, QR, Chip) across Bottom & Top Shelves
- 5 Central Drop-Off Color Zones
- Kinematics and Lift Height constants
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional


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
    color: str
    description: str
    center_pose: Pose3D
    bounds: Tuple[float, float, float, float]  # (xmin, xmax, ymin, ymax)
    approach_pose: Pose2D


# ==============================================================================
# 1. ROBOT SPAWN POSE & CONSTANTS
# ==============================================================================
ROBOT_SPAWN = Pose3D(x=-0.985, y=0.640, z=0.080, yaw=3.14159265)

# Kinematic offsets
LIFT_ARM_LATERAL_OFFSET = 0.00827  # 8.27mm offset of lift_arm_joint in base_link
FORK_REACH_DISTANCE = 0.2308       # 23.08cm distance from robot center to fork tips

# Lift Height settings (meters)
LIFT_HEIGHT_TRANSIT = 0.015        # Safe ground clearance while driving
LIFT_HEIGHT_LEVEL1_INSERT = 0.0295 # Align fork with bottom shelf pallet opening
LIFT_HEIGHT_LEVEL1_CARRY = 0.0700  # Elevated above bottom shelf
LIFT_HEIGHT_LEVEL2_INSERT = 0.1495 # Align fork with middle shelf pallet opening
LIFT_HEIGHT_LEVEL2_CARRY = 0.1850  # Elevated above middle shelf


# ==============================================================================
# 2. STORAGE RACKS (4 Racks on Team Side: X < 0)
# ==============================================================================
STORAGE_RACKS: Dict[str, StorageRack] = {
    "rack_left_top": StorageRack(
        name="rack_left_top",
        description="Kệ góc trên bên trái",
        pose=Pose3D(x=-1.893, y=-0.649, z=0.0025, yaw=1.5708),
        approach_pose=Pose2D(x=-1.600, y=-0.649, yaw=3.1416),
    ),
    "rack_left_mid": StorageRack(
        name="rack_left_mid",
        description="Kệ ở giữa bên trái",
        pose=Pose3D(x=-1.894, y=-0.006, z=0.0025, yaw=1.5708),
        approach_pose=Pose2D(x=-1.600, y=-0.006, yaw=3.1416),
    ),
    "rack_left_bot": StorageRack(
        name="rack_left_bot",
        description="Kệ góc dưới bên trái",
        pose=Pose3D(x=-1.894, y=0.641, z=0.0025, yaw=1.5708),
        approach_pose=Pose2D(x=-1.600, y=0.641, yaw=3.1416),
    ),
    "rack_bot_mid_left": StorageRack(
        name="rack_bot_mid_left",
        description="Kệ ở cạnh dưới (giữa trái)",
        pose=Pose3D(x=-0.490, y=0.895, z=0.0025, yaw=0.0000),
        approach_pose=Pose2D(x=-0.490, y=0.650, yaw=1.5708),
    ),
}


# ==============================================================================
# 3. PALLETS (16 Pallets across 4 Racks)
# ==============================================================================
PALLETS: Dict[str, Pallet] = {
    # --- RACK LEFT TOP ---
    "pallet_aluminum_rack_left_top_b_left": Pallet(
        name="pallet_aluminum_rack_left_top_b_left",
        rack="rack_left_top", shelf="bottom", slot="left", item_type="aluminum", block_id=0,
        pose=Pose3D(x=-1.893, y=-0.709, z=0.0285, yaw=1.5708)
    ),
    "pallet_cpu_rack_left_top_b_right": Pallet(
        name="pallet_cpu_rack_left_top_b_right",
        rack="rack_left_top", shelf="bottom", slot="right", item_type="cpu", block_id=1,
        pose=Pose3D(x=-1.893, y=-0.589, z=0.0285, yaw=1.5708)
    ),
    "pallet_qr_rack_left_top_t_left": Pallet(
        name="pallet_qr_rack_left_top_t_left",
        rack="rack_left_top", shelf="top", slot="left", item_type="qr", block_id=2,
        pose=Pose3D(x=-1.893, y=-0.709, z=0.1485, yaw=1.5708)
    ),
    "pallet_chip_rack_left_top_t_right": Pallet(
        name="pallet_chip_rack_left_top_t_right",
        rack="rack_left_top", shelf="top", slot="right", item_type="chip", block_id=3,
        pose=Pose3D(x=-1.893, y=-0.589, z=0.1485, yaw=1.5708)
    ),

    # --- RACK LEFT MID ---
    "pallet_qr_rack_left_mid_b_left": Pallet(
        name="pallet_qr_rack_left_mid_b_left",
        rack="rack_left_mid", shelf="bottom", slot="left", item_type="qr", block_id=2,
        pose=Pose3D(x=-1.894, y=-0.066, z=0.0285, yaw=1.5708)
    ),
    "pallet_chip_rack_left_mid_b_right": Pallet(
        name="pallet_chip_rack_left_mid_b_right",
        rack="rack_left_mid", shelf="bottom", slot="right", item_type="chip", block_id=3,
        pose=Pose3D(x=-1.894, y=0.054, z=0.0285, yaw=1.5708)
    ),
    "pallet_aluminum_rack_left_mid_t_left": Pallet(
        name="pallet_aluminum_rack_left_mid_t_left",
        rack="rack_left_mid", shelf="top", slot="left", item_type="aluminum", block_id=0,
        pose=Pose3D(x=-1.894, y=-0.066, z=0.1485, yaw=1.5708)
    ),
    "pallet_cpu_rack_left_mid_t_right": Pallet(
        name="pallet_cpu_rack_left_mid_t_right",
        rack="rack_left_mid", shelf="top", slot="right", item_type="cpu", block_id=1,
        pose=Pose3D(x=-1.894, y=0.054, z=0.1485, yaw=1.5708)
    ),

    # --- RACK LEFT BOT ---
    "pallet_aluminum_rack_left_bot_b_left": Pallet(
        name="pallet_aluminum_rack_left_bot_b_left",
        rack="rack_left_bot", shelf="bottom", slot="left", item_type="aluminum", block_id=0,
        pose=Pose3D(x=-1.894, y=0.581, z=0.0285, yaw=1.5708)
    ),
    "pallet_cpu_rack_left_bot_b_right": Pallet(
        name="pallet_cpu_rack_left_bot_b_right",
        rack="rack_left_bot", shelf="bottom", slot="right", item_type="cpu", block_id=1,
        pose=Pose3D(x=-1.894, y=0.701, z=0.0285, yaw=1.5708)
    ),
    "pallet_qr_rack_left_bot_t_left": Pallet(
        name="pallet_qr_rack_left_bot_t_left",
        rack="rack_left_bot", shelf="top", slot="left", item_type="qr", block_id=2,
        pose=Pose3D(x=-1.894, y=0.581, z=0.1485, yaw=1.5708)
    ),
    "pallet_chip_rack_left_bot_t_right": Pallet(
        name="pallet_chip_rack_left_bot_t_right",
        rack="rack_left_bot", shelf="top", slot="right", item_type="chip", block_id=3,
        pose=Pose3D(x=-1.894, y=0.701, z=0.1485, yaw=1.5708)
    ),

    # --- RACK BOT MID LEFT ---
    "pallet_qr_rack_bot_mid_left_b_left": Pallet(
        name="pallet_qr_rack_bot_mid_left_b_left",
        rack="rack_bot_mid_left", shelf="bottom", slot="left", item_type="qr", block_id=2,
        pose=Pose3D(x=-0.550, y=0.895, z=0.0285, yaw=0.0000)
    ),
    "pallet_chip_rack_bot_mid_left_b_right": Pallet(
        name="pallet_chip_rack_bot_mid_left_b_right",
        rack="rack_bot_mid_left", shelf="bottom", slot="right", item_type="chip", block_id=3,
        pose=Pose3D(x=-0.430, y=0.895, z=0.0285, yaw=0.0000)
    ),
    "pallet_aluminum_rack_bot_mid_left_t_left": Pallet(
        name="pallet_aluminum_rack_bot_mid_left_t_left",
        rack="rack_bot_mid_left", shelf="top", slot="left", item_type="aluminum", block_id=0,
        pose=Pose3D(x=-0.550, y=0.895, z=0.1485, yaw=0.0000)
    ),
    "pallet_cpu_rack_bot_mid_left_t_right": Pallet(
        name="pallet_cpu_rack_bot_mid_left_t_right",
        rack="rack_bot_mid_left", shelf="top", slot="right", item_type="cpu", block_id=1,
        pose=Pose3D(x=-0.430, y=0.895, z=0.1485, yaw=0.0000)
    ),
}


# ==============================================================================
# 4. CENTRAL DROP-OFF ZONES (5 Colored Zones at center divider: X < 0)
# ==============================================================================
DROPOFF_ZONES: Dict[str, DropOffZone] = {
    "zone_1_blue": DropOffZone(
        name="zone_1_blue", index=1, color="blue", description="Ô số 1 (Viền Xanh dương)",
        center_pose=Pose3D(x=-0.1271, y=0.6417, z=0.0025, yaw=0.0),
        bounds=(-0.255, 0.000, 0.513, 0.771),
        approach_pose=Pose2D(x=-0.420, y=0.642, yaw=0.0000)
    ),
    "zone_2_green": DropOffZone(
        name="zone_2_green", index=2, color="green", description="Ô số 2 (Viền Xanh lá)",
        center_pose=Pose3D(x=-0.1271, y=0.3138, z=0.0025, yaw=0.0),
        bounds=(-0.255, 0.000, 0.185, 0.443),
        approach_pose=Pose2D(x=-0.420, y=0.314, yaw=0.0000)
    ),
    "zone_3_white": DropOffZone(
        name="zone_3_white", index=3, color="white", description="Ô số 3 (Viền Trắng - Trung tâm)",
        center_pose=Pose3D(x=-0.1271, y=-0.0047, z=0.0025, yaw=0.0),
        bounds=(-0.255, 0.000, -0.134, 0.124),
        approach_pose=Pose2D(x=-0.420, y=0.000, yaw=0.0000)
    ),
    "zone_4_yellow": DropOffZone(
        name="zone_4_yellow", index=4, color="yellow", description="Ô số 4 (Viền Vàng)",
        center_pose=Pose3D(x=-0.1271, y=-0.3208, z=0.0025, yaw=0.0),
        bounds=(-0.255, 0.000, -0.450, -0.192),
        approach_pose=Pose2D(x=-0.420, y=-0.321, yaw=0.0000)
    ),
    "zone_5_red": DropOffZone(
        name="zone_5_red", index=5, color="red", description="Ô số 5 (Viền Đỏ)",
        center_pose=Pose3D(x=-0.1271, y=-0.6487, z=0.0025, yaw=0.0),
        bounds=(-0.255, 0.000, -0.778, -0.520),
        approach_pose=Pose2D(x=-0.420, y=-0.649, yaw=0.0000)
    ),
}


# ==============================================================================
# 5. QUERY & HELPER METHODS
# ==============================================================================
def get_pallets_by_type(item_type: str) -> List[Pallet]:
    """Return all pallets matching item type ('aluminum', 'cpu', 'qr', 'chip')."""
    return [p for p in PALLETS.values() if p.item_type.lower() == item_type.lower()]


def get_pallets_by_rack(rack_name: str) -> List[Pallet]:
    """Return all pallets placed on a specific rack."""
    return [p for p in PALLETS.values() if p.rack == rack_name]


def get_dropoff_by_color(color_name: str) -> Optional[DropOffZone]:
    """Return dropoff zone for a color ('blue', 'green', 'white', 'yellow', 'red')."""
    target = color_name.lower()
    for zone in DROPOFF_ZONES.values():
        if zone.color == target or zone.name == target:
            return zone
    return None


def get_pallet_by_location(rack: str, shelf: str, slot: str) -> Optional[Pallet]:
    """Find specific pallet by (rack_name, shelf, slot)."""
    for p in PALLETS.values():
        if p.rack == rack and p.shelf == shelf and p.slot == slot:
            return p
    return None

