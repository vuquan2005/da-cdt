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
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import yaml


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


@dataclass(frozen=True)
class LineIntersection:
    name: str
    description: str
    pose: Pose2D


# ==============================================================================
# CONFIG LOADER: YAML SINGLE SOURCE OF TRUTH
# ==============================================================================
def find_arena_config_path() -> str:
    """Finds the absolute path to arena_coordinates.yaml."""
    env_path = os.environ.get('ARENA_COORDINATES_YAML')
    if env_path and os.path.exists(env_path):
        return env_path

    try:
        from ament_index_python.packages import get_package_share_directory
        share_path = os.path.join(get_package_share_directory('robot0_navigation'), 'config', 'arena_coordinates.yaml')
        if os.path.exists(share_path):
            return share_path
    except Exception:
        pass

    module_dir = os.path.dirname(os.path.abspath(__file__))
    candidate_paths = [
        os.path.abspath(os.path.join(module_dir, '..', '..', 'config', 'arena_coordinates.yaml')),
        os.path.abspath(os.path.join(module_dir, '..', 'config', 'arena_coordinates.yaml')),
        os.path.abspath(os.path.join(module_dir, 'config', 'arena_coordinates.yaml')),
        '/workspaces/ros-cdt/src/robot0_navigation/config/arena_coordinates.yaml',
    ]

    for path in candidate_paths:
        if os.path.exists(path):
            return path

    raise FileNotFoundError("Could not locate arena_coordinates.yaml configuration file.")


def load_arena_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """Loads and parses arena_coordinates.yaml into raw dictionary."""
    if config_path is None:
        config_path = find_arena_config_path()

    with open(config_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return data or {}


# Load data from YAML
_RAW_CONFIG = load_arena_config()

# 1. Robot Spawn Pose & Kinematics
_spawn_raw = _RAW_CONFIG.get('robot_spawn', {})
ROBOT_SPAWN = Pose3D(
    x=float(_spawn_raw.get('x', -0.985)),
    y=float(_spawn_raw.get('y', 0.640)),
    z=float(_spawn_raw.get('z', 0.080)),
    yaw=float(_spawn_raw.get('yaw', math.pi))
)

_kinematics_raw = _RAW_CONFIG.get('kinematics', {})
LIFT_ARM_LATERAL_OFFSET = float(_kinematics_raw.get('lift_arm_lateral_offset', 0.0))
FORK_REACH_DISTANCE = float(_kinematics_raw.get('fork_reach_distance', 0.2308))
WHEEL_RADIUS = float(_kinematics_raw.get('wheel_radius', 0.0487))
HALF_WHEELBASE_LX = float(_kinematics_raw.get('half_wheelbase_lx', 0.1000))
HALF_TRACK_LY = float(_kinematics_raw.get('half_track_ly', 0.1539))

# Lift Height Settings & Control limits
_lift_raw = _RAW_CONFIG.get('lift_heights', {})
LIFT_HEIGHT_TRANSIT = float(_lift_raw.get('transit', 0.0150))
LIFT_HEIGHT_LEVEL1_INSERT = float(_lift_raw.get('level1_insert', 0.0295))
LIFT_HEIGHT_LEVEL1_CARRY = float(_lift_raw.get('level1_carry', 0.0700))
LIFT_HEIGHT_LEVEL2_INSERT = float(_lift_raw.get('level2_insert', 0.1495))
LIFT_HEIGHT_LEVEL2_CARRY = float(_lift_raw.get('level2_carry', 0.1850))
LIFT_HEIGHT_DROPOFF = float(_lift_raw.get('dropoff', 0.0000))

_lift_ctrl_raw = _RAW_CONFIG.get('lift_control', {})
LIFT_HEIGHT_TOLERANCE = float(_lift_ctrl_raw.get('tolerance', 0.003))
LIFT_TIMEOUT_SEC = float(_lift_ctrl_raw.get('timeout_sec', 10.0))

# 2. Storage Racks
STORAGE_RACKS: Dict[str, StorageRack] = {}
for _k, _v in _RAW_CONFIG.get('storage_racks', {}).items():
    _pose_dict = _v.get('pose', {})
    _app_dict = _v.get('approach_pose', {})
    STORAGE_RACKS[_k] = StorageRack(
        name=_k,
        description=_v.get('description', ''),
        pose=Pose3D(
            x=float(_pose_dict.get('x', 0.0)),
            y=float(_pose_dict.get('y', 0.0)),
            z=float(_pose_dict.get('z', 0.0)),
            yaw=float(_pose_dict.get('yaw', 0.0))
        ),
        approach_pose=Pose2D(
            x=float(_app_dict.get('x', 0.0)),
            y=float(_app_dict.get('y', 0.0)),
            yaw=float(_app_dict.get('yaw', 0.0))
        )
    )

# 3. Pallets
PALLETS: Dict[str, Pallet] = {}
for _k, _v in _RAW_CONFIG.get('pallets', {}).items():
    _pose_dict = _v.get('pose', {})
    PALLETS[_k] = Pallet(
        name=_k,
        rack=_v.get('rack', ''),
        shelf=_v.get('shelf', 'bottom'),
        slot=_v.get('slot', 'left'),
        item_type=_v.get('item_type', ''),
        block_id=int(_v.get('block_id', 0)),
        pose=Pose3D(
            x=float(_pose_dict.get('x', 0.0)),
            y=float(_pose_dict.get('y', 0.0)),
            z=float(_pose_dict.get('z', 0.0)),
            yaw=float(_pose_dict.get('yaw', 0.0))
        )
    )

# 4. Drop-off Zones
DROPOFF_ZONES: Dict[str, DropOffZone] = {}
for _k, _v in _RAW_CONFIG.get('dropoff_zones', {}).items():
    _cp_dict = _v.get('center_pose', {})
    _ap_dict = _v.get('approach_pose', {})
    _idx = _v.get('index')
    if _idx is None:
        try:
            _idx = int(_k.split('_')[-1])
        except Exception:
            _idx = 0
    DROPOFF_ZONES[_k] = DropOffZone(
        name=_k,
        index=int(_idx),
        item_type=_v.get('item_type', ''),
        description=_v.get('description', ''),
        center_pose=Pose3D(
            x=float(_cp_dict.get('x', 0.0)),
            y=float(_cp_dict.get('y', 0.0)),
            z=float(_cp_dict.get('z', 0.0)),
            yaw=float(_cp_dict.get('yaw', 0.0))
        ),
        approach_pose=Pose2D(
            x=float(_ap_dict.get('x', 0.0)),
            y=float(_ap_dict.get('y', 0.0)),
            yaw=float(_ap_dict.get('yaw', 0.0))
        )
    )

# 5. Line Grid Intersections
LINE_INTERSECTIONS: Dict[str, LineIntersection] = {}
for _k, _v in _RAW_CONFIG.get('line_intersections', {}).items():
    _p = Pose2D(
        x=float(_v.get('x', 0.0)),
        y=float(_v.get('y', 0.0)),
        yaw=float(_v.get('yaw', 0.0))
    )
    _inter = LineIntersection(
        name=_k,
        description=_v.get('desc', ''),
        pose=_p
    )
    LINE_INTERSECTIONS[_k] = _inter
    LINE_INTERSECTIONS[_k.upper()] = _inter  # Dual indexing for uppercase and lowercase access


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


def calculate_pallet_pick_poses(pallet: Pallet) -> Tuple[Pose2D, Pose2D, Pose2D]:
    """
    Calculates the 3 key interaction poses for picking a pallet:
    1. staging_pose: Alignment pose in front of rack slot (X = -1.500m)
    2. insert_pose: Deep insertion pose under pallet (X = -1.645m)
    3. retract_pose: Backed out pose holding pallet (X = -1.500m)
    Compensates for lateral lift arm offset (LIFT_ARM_LATERAL_OFFSET).
    """
    aligned_y = pallet.pose.y + LIFT_ARM_LATERAL_OFFSET
    staging_pose = Pose2D(x=-1.500, y=aligned_y, yaw=math.pi)
    insert_pose = Pose2D(x=-1.645, y=aligned_y, yaw=math.pi)
    retract_pose = Pose2D(x=-1.500, y=aligned_y, yaw=math.pi)
    return staging_pose, insert_pose, retract_pose


def generate_approach_route(target_rack: str) -> list:
    """
    Generates strictly orthogonal waypoint route from Home Spawn to Rack Main-Line Intersection.
    Maintains constant heading (Yaw = pi) without unwanted rotational drift.
    """
    route = []
    if target_rack == 'rack_2':
        # Start (X=-0.985, Y=0.640, Yaw=pi) -> Switch Top (X=-0.400, Y=0.640, Yaw=pi)
        route.append(Pose2D(x=-0.400, y=0.640, yaw=math.pi))
        # -> Switch Bot (X=-0.400, Y=0.000, Yaw=pi)
        route.append(Pose2D(x=-0.400, y=0.000, yaw=math.pi))
        # -> Rack 2 West Line (X=-1.500, Y=0.000, Yaw=pi)
        route.append(Pose2D(x=-1.500, y=0.000, yaw=math.pi))
    else:
        # Start (X=-0.985, Y=0.640, Yaw=pi) -> Rack 1 West Line (X=-1.500, Y=0.640, Yaw=pi)
        route.append(Pose2D(x=-1.500, y=0.640, yaw=math.pi))

    return route


def generate_delivery_route(rack_name: str, dropoff_zone: Optional[DropOffZone]) -> list:
    """
    Generates strictly orthogonal delivery route:
    1. Backs out linearly from Rack Line to safe clearance track (X=-1.350m, Yaw=pi)
    2. Cleanly rotates in-place to Yaw = 0.0 (facing East)
    3. Travels pure orthogonal lines along horizontal and vertical trunks to Drop-off Zone.
    """
    route = []
    rack_y = 0.640 if rack_name == 'rack_1' else 0.000
    target_y = dropoff_zone.approach_pose.y if dropoff_zone else 0.640

    # 1. Back out straight West-facing to safe clearance track line
    route.append(Pose2D(x=-1.350, y=rack_y, yaw=math.pi))

    # 2. Rotate in-place cleanly to face East (Yaw = 0.0) at open track
    route.append(Pose2D(x=-1.350, y=rack_y, yaw=0.0))

    if rack_name == 'rack_1':
        # 3. Travel East along Row 1
        route.append(Pose2D(x=-0.985, y=0.640, yaw=0.0))
        route.append(Pose2D(x=-0.400, y=0.640, yaw=0.0))
        route.append(Pose2D(x=0.000, y=0.640, yaw=0.0))

        # 4. Travel down Central Distribution Trunk (Orthogonal Y movement)
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
        # 3. Travel East along Row 2
        route.append(Pose2D(x=-0.400, y=0.000, yaw=0.0))
        route.append(Pose2D(x=0.000, y=0.000, yaw=0.0))

        # 4. Travel up/down Central Distribution Trunk
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

    # 5. Branch out East directly into target Drop-off Zone
    if dropoff_zone:
        route.append(Pose2D(x=dropoff_zone.approach_pose.x, y=dropoff_zone.approach_pose.y, yaw=0.0))
    else:
        route.append(Pose2D(x=0.550, y=0.640, yaw=0.0))

    return route


def generate_return_home_route(current_dropoff_y: float) -> list:
    """
    Generates strictly orthogonal waypoint route from Drop-off area back to Home Base:
    1. Backs out straight to Central Trunk (Yaw=0.0)
    2. Moves along Central Trunk to North intersection (Y=0.640m, Yaw=0.0)
    3. Cleanly rotates in-place to Yaw = pi (facing West)
    4. Travels straight West along Row 1 back into Home Base.
    """
    route = []
    # 1. Back out West to Central Trunk
    route.append(Pose2D(x=0.000, y=current_dropoff_y, yaw=0.0))

    # 2. Travel up Central Trunk to North Intersection (Y=0.640m)
    if abs(current_dropoff_y - (-0.640)) < 0.05:
        route.append(Pose2D(x=0.000, y=-0.220, yaw=0.0))
        route.append(Pose2D(x=0.000, y=0.000, yaw=0.0))
        route.append(Pose2D(x=0.000, y=0.220, yaw=0.0))
        route.append(Pose2D(x=0.000, y=0.640, yaw=0.0))
    elif abs(current_dropoff_y - (-0.220)) < 0.05:
        route.append(Pose2D(x=0.000, y=0.000, yaw=0.0))
        route.append(Pose2D(x=0.000, y=0.220, yaw=0.0))
        route.append(Pose2D(x=0.000, y=0.640, yaw=0.0))
    elif abs(current_dropoff_y - 0.220) < 0.05:
        route.append(Pose2D(x=0.000, y=0.640, yaw=0.0))

    # 3. Rotate in-place to face West (Yaw = pi) at North Intersection
    route.append(Pose2D(x=0.000, y=0.640, yaw=math.pi))

    # 4. Travel straight West along Row 1 back to Start
    route.append(Pose2D(x=-0.400, y=0.640, yaw=math.pi))
    route.append(Pose2D(x=ROBOT_SPAWN.x, y=ROBOT_SPAWN.y, yaw=math.pi))
    return route


def generate_rack_to_rack_route(from_rack: str, to_rack: str) -> list:
    """
    Generates orthogonal route between Rack 1 and Rack 2 via the West Switch lane.
    """
    route = []
    if from_rack == 'rack_1' and to_rack == 'rack_2':
        # Rack 1 (X=-1.500, Y=0.640) -> Switch Top (X=-0.400, Y=0.640)
        route.append(Pose2D(x=-0.400, y=0.640, yaw=math.pi))
        # -> Switch Bot (X=-0.400, Y=0.000)
        route.append(Pose2D(x=-0.400, y=0.000, yaw=math.pi))
        # -> Rack 2 (X=-1.500, Y=0.000)
        route.append(Pose2D(x=-1.500, y=0.000, yaw=math.pi))
    elif from_rack == 'rack_2' and to_rack == 'rack_1':
        # Rack 2 (X=-1.500, Y=0.000) -> Switch Bot (X=-0.400, Y=0.000)
        route.append(Pose2D(x=-0.400, y=0.000, yaw=math.pi))
        # -> Switch Top (X=-0.400, Y=0.640)
        route.append(Pose2D(x=-0.400, y=0.640, yaw=math.pi))
        # -> Rack 1 (X=-1.500, Y=0.640)
        route.append(Pose2D(x=-1.500, y=0.640, yaw=math.pi))
    return route


def generate_return_home_from_rack_route(rack_name: str) -> list:
    """
    Generates route from Rack 1 or Rack 2 back to Start Base when aborting/not found.
    """
    route = []
    if rack_name == 'rack_2':
        # Rack 2 (X=-1.500, Y=0.000) -> Switch Bot (X=-0.400, Y=0.000)
        route.append(Pose2D(x=-0.400, y=0.000, yaw=math.pi))
        # -> Switch Top (X=-0.400, Y=0.640)
        route.append(Pose2D(x=-0.400, y=0.640, yaw=math.pi))
        # -> Home (X=-0.985, Y=0.640)
        route.append(Pose2D(x=ROBOT_SPAWN.x, y=ROBOT_SPAWN.y, yaw=math.pi))
    else:
        # Rack 1 (X=-1.500, Y=0.640) -> Home (X=-0.985, Y=0.640)
        route.append(Pose2D(x=ROBOT_SPAWN.x, y=ROBOT_SPAWN.y, yaw=math.pi))
    return route
