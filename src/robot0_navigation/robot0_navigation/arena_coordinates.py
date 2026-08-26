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
class Intersection:
    name: str
    description: str
    junction_type: str  # 'CROSS', 'T_JUNCTION', 'CORNER', 'DEAD_END'
    pose: Pose3D
    neighbors: Tuple[str, ...] = ()
    branches: Tuple[str, ...] = ()

    @property
    def x(self) -> float:
        return self.pose.x

    @property
    def y(self) -> float:
        return self.pose.y

    @property
    def yaw(self) -> float:
        return self.pose.yaw


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
# 5. LINE NETWORK INTERSECTIONS / JUNCTIONS (TỌA ĐỘ CÁC NÚT GIAO SA BÀN)
# ==============================================================================
INTERSECTIONS: Dict[str, Intersection] = {
    # --- 5.1 Nút giao tiếp cận Kệ hàng (Storage Rack Approach Junctions: X < 0) ---
    "junc_rack_left_bot": Intersection(
        name="junc_rack_left_bot",
        description="Nút giao tiếp cận Kệ trái dưới (rack_left_bot - Hàng 1 / Spawn side)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-1.840, y=0.643, z=0.0025, yaw=0.0000),
        neighbors=("junc_outer_left_row1",),
        branches=("east", "north", "south"),
    ),
    "junc_rack_left_mid": Intersection(
        name="junc_rack_left_mid",
        description="Nút giao tiếp cận Kệ trái giữa (rack_left_mid - Hàng 3 / Trung tâm)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-1.840, y=-0.003, z=0.0025, yaw=0.0000),
        neighbors=("junc_outer_left_row3",),
        branches=("east", "north", "south"),
    ),
    "junc_rack_left_top": Intersection(
        name="junc_rack_left_top",
        description="Nút giao tiếp cận Kệ trái trên (rack_left_top - Hàng 5)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-1.840, y=-0.650, z=0.0025, yaw=0.0000),
        neighbors=("junc_outer_left_row5",),
        branches=("east", "north", "south"),
    ),
    "junc_rack_bot_mid_left": Intersection(
        name="junc_rack_bot_mid_left",
        description="Nút giao tiếp cận Kệ cạnh trên giữa trái (rack_bot_mid_left)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-0.490, y=0.848, z=0.0025, yaw=-1.5708),
        neighbors=("junc_inner_left_row1",),
        branches=("south", "east", "west"),
    ),

    # --- 5.2 Nút giao Trục dọc ngoài bên trái (Outer Left Corridor: X = -1.480m) ---
    "junc_outer_left_row1": Intersection(
        name="junc_outer_left_row1",
        description="Giao lộ trục dọc ngoài - Hàng 1 (Nối rack_left_bot với trục trong)",
        junction_type="CROSS",
        pose=Pose3D(x=-1.480, y=0.643, z=0.0025, yaw=0.0000),
        neighbors=("junc_rack_left_bot", "junc_outer_left_row3", "junc_inner_left_row1"),
        branches=("west", "south", "east"),
    ),
    "junc_outer_left_row3": Intersection(
        name="junc_outer_left_row3",
        description="Giao lộ trục dọc ngoài - Hàng 3 (Nối rack_left_mid với trục trong)",
        junction_type="CROSS",
        pose=Pose3D(x=-1.480, y=-0.003, z=0.0025, yaw=0.0000),
        neighbors=("junc_rack_left_mid", "junc_outer_left_row1", "junc_outer_left_row5", "junc_inner_left_row3"),
        branches=("west", "north", "south", "east"),
    ),
    "junc_outer_left_row5": Intersection(
        name="junc_outer_left_row5",
        description="Giao lộ trục dọc ngoài - Hàng 5 (Nối rack_left_top với trục trong)",
        junction_type="CROSS",
        pose=Pose3D(x=-1.480, y=-0.650, z=0.0025, yaw=0.0000),
        neighbors=("junc_rack_left_top", "junc_outer_left_row3", "junc_inner_left_row5"),
        branches=("west", "north", "east"),
    ),

    # --- 5.3 Nút giao Trục dọc trong bên trái (Inner Left Corridor: X = -0.490m) ---
    "junc_inner_left_row1": Intersection(
        name="junc_inner_left_row1",
        description="Giao lộ chính trục trong - Hàng 1 (Khu vực Spawn / Dẫn vào Zone 1 Blue)",
        junction_type="CROSS",
        pose=Pose3D(x=-0.490, y=0.643, z=0.0025, yaw=0.0000),
        neighbors=("junc_rack_bot_mid_left", "junc_outer_left_row1", "junc_inner_left_row2", "junc_dropoff_zone1"),
        branches=("north", "south", "west", "east"),
    ),
    "junc_inner_left_row2": Intersection(
        name="junc_inner_left_row2",
        description="Ngã 3 trục trong - Rẽ vào Hàng 2 (Dẫn vào Zone 2 Green)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-0.490, y=0.316, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row1", "junc_inner_left_row3", "junc_dropoff_zone2"),
        branches=("north", "south", "east"),
    ),
    "junc_inner_left_row3": Intersection(
        name="junc_inner_left_row3",
        description="Giao lộ chính trục trong - Hàng 3 (Trục ngang trung tâm / Dẫn vào Zone 3 White)",
        junction_type="CROSS",
        pose=Pose3D(x=-0.490, y=-0.003, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row2", "junc_inner_left_row4", "junc_outer_left_row3", "junc_dropoff_zone3"),
        branches=("north", "south", "west", "east"),
    ),
    "junc_inner_left_row4": Intersection(
        name="junc_inner_left_row4",
        description="Ngã 3 trục trong - Rẽ vào Hàng 4 (Dẫn vào Zone 4 Yellow)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-0.490, y=-0.323, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row3", "junc_inner_left_row5", "junc_dropoff_zone4"),
        branches=("north", "south", "east"),
    ),
    "junc_inner_left_row5": Intersection(
        name="junc_inner_left_row5",
        description="Giao lộ chính trục trong - Hàng 5 (Dẫn vào Zone 5 Red)",
        junction_type="CROSS",
        pose=Pose3D(x=-0.490, y=-0.650, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row4", "junc_outer_left_row5", "junc_dropoff_zone5"),
        branches=("north", "west", "east"),
    ),

    # --- 5.4 Nút giao Cửa ngõ tiếp cận Ô trả hàng (Drop-Off Entry Junctions: X = -0.241m) ---
    "junc_dropoff_zone1": Intersection(
        name="junc_dropoff_zone1",
        description="Cửa ngõ tiếp cận Ô số 1 (Zone 1 Blue - Xanh dương)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-0.241, y=0.643, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row1", "junc_center_row1"),
        branches=("west", "east", "north", "south"),
    ),
    "junc_dropoff_zone2": Intersection(
        name="junc_dropoff_zone2",
        description="Cửa ngõ tiếp cận Ô số 2 (Zone 2 Green - Xanh lá)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-0.241, y=0.316, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row2", "junc_center_row2"),
        branches=("west", "east", "north", "south"),
    ),
    "junc_dropoff_zone3": Intersection(
        name="junc_dropoff_zone3",
        description="Cửa ngõ tiếp cận Ô số 3 (Zone 3 White - Trắng trung tâm)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-0.241, y=-0.003, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row3", "junc_center_row3"),
        branches=("west", "east"),
    ),
    "junc_dropoff_zone4": Intersection(
        name="junc_dropoff_zone4",
        description="Cửa ngõ tiếp cận Ô số 4 (Zone 4 Yellow - Vàng)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-0.241, y=-0.323, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row4", "junc_center_row4"),
        branches=("west", "east", "north", "south"),
    ),
    "junc_dropoff_zone5": Intersection(
        name="junc_dropoff_zone5",
        description="Cửa ngõ tiếp cận Ô số 5 (Zone 5 Red - Đỏ)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=-0.241, y=-0.650, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_left_row5", "junc_center_row5"),
        branches=("west", "east", "north", "south"),
    ),

    # --- 5.5 Nút giao Trục phân cách Trung tâm (Center Divider Junctions: X = 0.000m) ---
    "junc_center_row1": Intersection(
        name="junc_center_row1",
        description="Giao lộ trục phân cách trung tâm - Hàng 1",
        junction_type="CROSS",
        pose=Pose3D(x=0.000, y=0.643, z=0.0025, yaw=0.0000),
        neighbors=("junc_dropoff_zone1", "junc_center_row2", "junc_dropoff_right_zone1"),
        branches=("west", "south", "east"),
    ),
    "junc_center_row2": Intersection(
        name="junc_center_row2",
        description="Giao lộ trục phân cách trung tâm - Hàng 2",
        junction_type="CROSS",
        pose=Pose3D(x=0.000, y=0.316, z=0.0025, yaw=0.0000),
        neighbors=("junc_center_row1", "junc_dropoff_zone2", "junc_center_row3", "junc_dropoff_right_zone2"),
        branches=("north", "south", "west", "east"),
    ),
    "junc_center_row3": Intersection(
        name="junc_center_row3",
        description="Giao lộ trục phân cách trung tâm - Hàng 3 (Tâm tuyệt đối sa bàn)",
        junction_type="CROSS",
        pose=Pose3D(x=0.000, y=-0.003, z=0.0025, yaw=0.0000),
        neighbors=("junc_center_row2", "junc_dropoff_zone3", "junc_center_row4", "junc_dropoff_right_zone3"),
        branches=("north", "south", "west", "east"),
    ),
    "junc_center_row4": Intersection(
        name="junc_center_row4",
        description="Giao lộ trục phân cách trung tâm - Hàng 4",
        junction_type="CROSS",
        pose=Pose3D(x=0.000, y=-0.323, z=0.0025, yaw=0.0000),
        neighbors=("junc_center_row3", "junc_dropoff_zone4", "junc_center_row5", "junc_dropoff_right_zone4"),
        branches=("north", "south", "west", "east"),
    ),
    "junc_center_row5": Intersection(
        name="junc_center_row5",
        description="Giao lộ trục phân cách trung tâm - Hàng 5",
        junction_type="CROSS",
        pose=Pose3D(x=0.000, y=-0.650, z=0.0025, yaw=0.0000),
        neighbors=("junc_center_row4", "junc_dropoff_zone5", "junc_dropoff_right_zone5"),
        branches=("north", "west", "east"),
    ),

    # --- 5.6 Nút giao Nửa sân đối diện / bên phải (Opponent / Right Side: X > 0) ---
    "junc_dropoff_right_zone1": Intersection(
        name="junc_dropoff_right_zone1",
        description="Cửa ngõ tiếp cận bên phải Ô số 1",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=0.260, y=0.643, z=0.0025, yaw=3.1416),
        neighbors=("junc_center_row1", "junc_inner_right_row1"),
        branches=("west", "east", "north", "south"),
    ),
    "junc_dropoff_right_zone2": Intersection(
        name="junc_dropoff_right_zone2",
        description="Cửa ngõ tiếp cận bên phải Ô số 2",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=0.260, y=0.316, z=0.0025, yaw=3.1416),
        neighbors=("junc_center_row2", "junc_inner_right_row2"),
        branches=("west", "east", "north", "south"),
    ),
    "junc_dropoff_right_zone3": Intersection(
        name="junc_dropoff_right_zone3",
        description="Cửa ngõ tiếp cận bên phải Ô số 3",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=0.260, y=-0.003, z=0.0025, yaw=3.1416),
        neighbors=("junc_center_row3", "junc_inner_right_row3"),
        branches=("west", "east"),
    ),
    "junc_dropoff_right_zone4": Intersection(
        name="junc_dropoff_right_zone4",
        description="Cửa ngõ tiếp cận bên phải Ô số 4",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=0.260, y=-0.323, z=0.0025, yaw=3.1416),
        neighbors=("junc_center_row4", "junc_inner_right_row4"),
        branches=("west", "east", "north", "south"),
    ),
    "junc_dropoff_right_zone5": Intersection(
        name="junc_dropoff_right_zone5",
        description="Cửa ngõ tiếp cận bên phải Ô số 5",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=0.260, y=-0.650, z=0.0025, yaw=3.1416),
        neighbors=("junc_center_row5", "junc_inner_right_row5"),
        branches=("west", "east", "north", "south"),
    ),
    "junc_inner_right_row1": Intersection(
        name="junc_inner_right_row1",
        description="Giao lộ chính trục trong bên phải - Hàng 1",
        junction_type="CROSS",
        pose=Pose3D(x=0.505, y=0.643, z=0.0025, yaw=0.0000),
        neighbors=("junc_dropoff_right_zone1", "junc_inner_right_row2", "junc_outer_right_row1"),
        branches=("west", "south", "east"),
    ),
    "junc_inner_right_row2": Intersection(
        name="junc_inner_right_row2",
        description="Ngã 3 trục trong bên phải - Hàng 2",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=0.505, y=0.316, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_right_row1", "junc_inner_right_row3", "junc_dropoff_right_zone2"),
        branches=("north", "south", "west"),
    ),
    "junc_inner_right_row3": Intersection(
        name="junc_inner_right_row3",
        description="Giao lộ chính trục trong bên phải - Hàng 3",
        junction_type="CROSS",
        pose=Pose3D(x=0.505, y=-0.003, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_right_row2", "junc_inner_right_row4", "junc_outer_right_row3", "junc_dropoff_right_zone3"),
        branches=("north", "south", "west", "east"),
    ),
    "junc_inner_right_row4": Intersection(
        name="junc_inner_right_row4",
        description="Ngã 3 trục trong bên phải - Hàng 4",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=0.505, y=-0.323, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_right_row3", "junc_inner_right_row5", "junc_dropoff_right_zone4"),
        branches=("north", "south", "west"),
    ),
    "junc_inner_right_row5": Intersection(
        name="junc_inner_right_row5",
        description="Giao lộ chính trục trong bên phải - Hàng 5",
        junction_type="CROSS",
        pose=Pose3D(x=0.505, y=-0.650, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_right_row4", "junc_outer_right_row5", "junc_rack_bot_mid_right", "junc_dropoff_right_zone5"),
        branches=("north", "south", "west", "east"),
    ),
    "junc_rack_bot_mid_right": Intersection(
        name="junc_rack_bot_mid_right",
        description="Nút giao tiếp cận Kệ cạnh dưới bên phải (rack_bot_mid_right)",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=0.505, y=-0.852, z=0.0025, yaw=1.5708),
        neighbors=("junc_inner_right_row5",),
        branches=("north", "east", "west"),
    ),
    "junc_outer_right_row1": Intersection(
        name="junc_outer_right_row1",
        description="Giao lộ trục dọc ngoài bên phải - Hàng 1",
        junction_type="CROSS",
        pose=Pose3D(x=1.497, y=0.643, z=0.0025, yaw=0.0000),
        neighbors=("junc_inner_right_row1", "junc_outer_right_row3", "junc_rack_right_bot"),
        branches=("west", "south", "east"),
    ),
    "junc_outer_right_row3": Intersection(
        name="junc_outer_right_row3",
        description="Giao lộ trục dọc ngoài bên phải - Hàng 3",
        junction_type="CROSS",
        pose=Pose3D(x=1.497, y=-0.003, z=0.0025, yaw=0.0000),
        neighbors=("junc_outer_right_row1", "junc_outer_right_row5", "junc_inner_right_row3", "junc_rack_right_mid"),
        branches=("north", "south", "west", "east"),
    ),
    "junc_outer_right_row5": Intersection(
        name="junc_outer_right_row5",
        description="Giao lộ trục dọc ngoài bên phải - Hàng 5",
        junction_type="CROSS",
        pose=Pose3D(x=1.497, y=-0.650, z=0.0025, yaw=0.0000),
        neighbors=("junc_outer_right_row3", "junc_inner_right_row5", "junc_rack_right_top"),
        branches=("north", "west", "east"),
    ),
    "junc_rack_right_bot": Intersection(
        name="junc_rack_right_bot",
        description="Nút giao tiếp cận Kệ phải dưới",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=1.856, y=0.643, z=0.0025, yaw=3.1416),
        neighbors=("junc_outer_right_row1",),
        branches=("west", "north", "south"),
    ),
    "junc_rack_right_mid": Intersection(
        name="junc_rack_right_mid",
        description="Nút giao tiếp cận Kệ phải giữa",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=1.856, y=-0.003, z=0.0025, yaw=3.1416),
        neighbors=("junc_outer_right_row3",),
        branches=("west", "north", "south"),
    ),
    "junc_rack_right_top": Intersection(
        name="junc_rack_right_top",
        description="Nút giao tiếp cận Kệ phải trên",
        junction_type="T_JUNCTION",
        pose=Pose3D(x=1.856, y=-0.650, z=0.0025, yaw=3.1416),
        neighbors=("junc_outer_right_row5",),
        branches=("west", "north", "south"),
    ),
}


# ==============================================================================
# 6. QUERY & HELPER METHODS
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


def get_intersection(name: str) -> Optional[Intersection]:
    """Retrieve intersection definition by its unique name."""
    return INTERSECTIONS.get(name)


def get_intersections_by_type(junction_type: str) -> List[Intersection]:
    """Return all intersections matching given junction type ('CROSS', 'T_JUNCTION', etc.)."""
    target = junction_type.upper()
    return [j for j in INTERSECTIONS.values() if j.junction_type.upper() == target]


def get_nearest_intersection(x: float, y: float) -> Tuple[str, Intersection, float]:
    """
    Find the closest arena intersection to given world coordinates (x, y).
    Returns (intersection_name, intersection_object, distance_in_meters).
    """
    best_name = ""
    best_junc = None
    best_dist = float("inf")
    for name, junc in INTERSECTIONS.items():
        d = math.hypot(junc.pose.x - x, junc.pose.y - y)
        if d < best_dist:
            best_dist = d
            best_name = name
            best_junc = junc
    return best_name, best_junc, best_dist


def get_intersection_graph() -> Dict[str, Dict[str, float]]:
    """Build adjacency graph mapping intersection_name -> {neighbor_name: distance_in_meters}."""
    graph: Dict[str, Dict[str, float]] = {}
    for name, junc in INTERSECTIONS.items():
        graph[name] = {}
        for nb_name in junc.neighbors:
            if nb_name in INTERSECTIONS:
                nb = INTERSECTIONS[nb_name]
                dist = math.hypot(nb.pose.x - junc.pose.x, nb.pose.y - junc.pose.y)
                graph[name][nb_name] = round(dist, 4)
    return graph


def find_shortest_intersection_path(start_node: str, end_node: str) -> List[str]:
    """Dijkstra shortest path algorithm between two intersection nodes on the arena grid."""
    if start_node not in INTERSECTIONS or end_node not in INTERSECTIONS:
        return []
    if start_node == end_node:
        return [start_node]

    graph = get_intersection_graph()
    import heapq
    queue = [(0.0, start_node, [start_node])]
    visited = set()

    while queue:
        cost, current, path = heapq.heappop(queue)
        if current == end_node:
            return path
        if current in visited:
            continue
        visited.add(current)

        for neighbor, weight in graph.get(current, {}).items():
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path + [neighbor]))
    return []

