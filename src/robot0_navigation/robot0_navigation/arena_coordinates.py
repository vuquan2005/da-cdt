#!/usr/bin/env python3
"""
Standard Arena Coordinates & Waypoint Map for Robot0 AMR Logistics Arena.

Defines:
- Home / Start Docking Station (0.0, 0.0)
- Pick-up Storage Rack (X = 1.5, Y = 0.0)
- Pallet Slots (Bottom Shelf: CPU & Aluminum, Top Shelf: Chip & QR)
- Drop-off Stations (Blue: North, Red: South, Green: West)
- Approach and Insertion Waypoints for Autonomous Pick & Place
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float  # Radians

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def angle_diff(self, other: "Pose2D") -> float:
        diff = other.yaw - self.yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


@dataclass(frozen=True)
class PalletSlot:
    name: str
    item_type: str        # 'cpu', 'aluminum', 'chip', 'qr'
    shelf_level: int      # 1 = Bottom (Z=0.025m), 2 = Top (Z=0.145m)
    slot_side: str        # 'left' (Y=-0.060) or 'right' (Y=+0.060)
    pallet_pose: Pose2D
    approach_pose: Pose2D # Robot waiting position before entering
    insert_pose: Pose2D   # Robot position when forks are fully inserted
    lift_height_approach: float  # Lift height before inserting (m)
    lift_height_carry: float     # Lift height while carrying (m)


@dataclass(frozen=True)
class DropOffStation:
    name: str
    color: str            # 'blue', 'red', 'green'
    station_pose: Pose2D
    approach_pose: Pose2D # Robot waiting position before entering
    insert_pose: Pose2D   # Robot position when placing pallet
    lift_height_place: float     # Lift height to release pallet (m)


# ==============================================================================
# 1. HOME / START DOCKING STATION
# ==============================================================================
HOME_BASE = Pose2D(x=0.0, y=0.0, yaw=0.0)


# ==============================================================================
# 2. PICK-UP STORAGE RACK & PALLET SLOTS
# ==============================================================================
# Storage Rack center: X = 1.5, Y = 0.0, Yaw = 1.5708 (open side facing -X)
# Fork length allows inserting when robot is at X ~ 1.38 - 1.40
PALLET_SLOTS: Dict[str, PalletSlot] = {
    "cpu_bottom_left": PalletSlot(
        name="cpu_bottom_left",
        item_type="cpu",
        shelf_level=1,
        slot_side="left",
        pallet_pose=Pose2D(x=1.50, y=-0.060, yaw=1.5708),
        approach_pose=Pose2D(x=1.10, y=-0.060, yaw=0.0000),
        insert_pose=Pose2D(x=1.38, y=-0.060, yaw=0.0000),
        lift_height_approach=0.000,
        lift_height_carry=0.050,
    ),
    "aluminum_bottom_right": PalletSlot(
        name="aluminum_bottom_right",
        item_type="aluminum",
        shelf_level=1,
        slot_side="right",
        pallet_pose=Pose2D(x=1.50, y=0.060, yaw=1.5708),
        approach_pose=Pose2D(x=1.10, y=0.060, yaw=0.0000),
        insert_pose=Pose2D(x=1.38, y=0.060, yaw=0.0000),
        lift_height_approach=0.000,
        lift_height_carry=0.050,
    ),
    "chip_top_left": PalletSlot(
        name="chip_top_left",
        item_type="chip",
        shelf_level=2,
        slot_side="left",
        pallet_pose=Pose2D(x=1.50, y=-0.060, yaw=1.5708),
        approach_pose=Pose2D(x=1.10, y=-0.060, yaw=0.0000),
        insert_pose=Pose2D(x=1.38, y=-0.060, yaw=0.0000),
        lift_height_approach=0.120,
        lift_height_carry=0.160,
    ),
    "qr_top_right": PalletSlot(
        name="qr_top_right",
        item_type="qr",
        shelf_level=2,
        slot_side="right",
        pallet_pose=Pose2D(x=1.50, y=0.060, yaw=1.5708),
        approach_pose=Pose2D(x=1.10, y=0.060, yaw=0.0000),
        insert_pose=Pose2D(x=1.38, y=0.060, yaw=0.0000),
        lift_height_approach=0.120,
        lift_height_carry=0.160,
    ),
}


# ==============================================================================
# 3. DROP-OFF / UNLOADING STATIONS
# ==============================================================================
DROPOFF_STATIONS: Dict[str, DropOffStation] = {
    "blue": DropOffStation(
        name="dropoff_blue",
        color="blue",
        station_pose=Pose2D(x=0.00, y=1.20, yaw=0.0000),
        approach_pose=Pose2D(x=0.00, y=0.80, yaw=1.5708),
        insert_pose=Pose2D(x=0.00, y=1.08, yaw=1.5708),
        lift_height_place=0.000,
    ),
    "red": DropOffStation(
        name="dropoff_red",
        color="red",
        station_pose=Pose2D(x=0.00, y=-1.20, yaw=3.1416),
        approach_pose=Pose2D(x=0.00, y=-0.80, yaw=-1.5708),
        insert_pose=Pose2D(x=0.00, y=-1.08, yaw=-1.5708),
        lift_height_place=0.000,
    ),
    "green": DropOffStation(
        name="dropoff_green",
        color="green",
        station_pose=Pose2D(x=-1.20, y=0.00, yaw=-1.5708),
        approach_pose=Pose2D(x=-0.80, y=0.00, yaw=3.1416),
        insert_pose=Pose2D(x=-1.08, y=0.00, yaw=3.1416),
        lift_height_place=0.000,
    ),
}


# ==============================================================================
# 4. QUERY FUNCTIONS
# ==============================================================================
def get_slot_by_type(item_type: str) -> Optional[PalletSlot]:
    """Find pallet slot by item type ('cpu', 'aluminum', 'chip', 'qr')."""
    target = item_type.lower()
    for slot in PALLET_SLOTS.values():
        if slot.item_type.lower() == target:
            return slot
    return None


def get_slot_by_shelf_and_side(shelf_level: int, slot_side: str) -> Optional[PalletSlot]:
    """Find pallet slot by shelf level (1 or 2) and side ('left' or 'right')."""
    target_side = slot_side.lower()
    for slot in PALLET_SLOTS.values():
        if slot.shelf_level == shelf_level and slot.slot_side.lower() == target_side:
            return slot
    return None


def get_dropoff_station(color_or_name: str) -> Optional[DropOffStation]:
    """Find drop-off station by color ('blue', 'red', 'green') or name."""
    target = color_or_name.lower().replace("dropoff_", "")
    return DROPOFF_STATIONS.get(target)


def get_default_dropoff_for_item(item_type: str) -> DropOffStation:
    """Return default drop-off station mapped to an item type."""
    mapping = {
        "cpu": "blue",       # CPU -> Blue Electronics Station
        "aluminum": "red",   # Aluminum -> Red Mechanical Station
        "chip": "green",     # Chip -> Green Inspection Station
        "qr": "blue",        # QR -> Blue Station
    }
    target_color = mapping.get(item_type.lower(), "blue")
    return DROPOFF_STATIONS[target_color]
