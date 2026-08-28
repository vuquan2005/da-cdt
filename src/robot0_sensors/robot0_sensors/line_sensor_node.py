#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Vector Geometric Dual Array Line Sensor Simulator Node for Robot0.

Simulates two N-channel optical/IR reflectance sensor arrays mounted at the
front (+X) and rear (-X) of the robot using pure mathematical vector geometry
(Zero GPU/Image dependency, ultra-fast 100Hz+ loop, sub-millimeter precision).

Features:
- Pure Geometric Line Segment Distance sampling (No floor image required)
- Dual Array (Front + Rear) with configurable eye count and spacing
- Continuous Smooth Analog & Binary Digital outputs
- Lateral deviation error & heading angle error
- Real-time Junction classification (CROSS, T_LEFT, T_RIGHT, NONE, LOST)
- Full RViz2 3D Sensor Markers & Status visualization
"""

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    DurabilityPolicy,
    ReliabilityPolicy,
    HistoryPolicy
)
import tf2_ros
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32, Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ==============================================================================
# VECTOR ARENA LINE DEFINITIONS (Meters)
# ==============================================================================
LINE_WIDTH = 0.025  # 25mm standard line width

# List of all track segments: ((x1, y1), (x2, y2))
TRACK_SEGMENTS: List[Tuple[Tuple[float, float], Tuple[float, float]]] = [
    # 1. Main Horizontal Lines
    ((-1.750, 0.640), (0.550, 0.640)),    # Lane 1 (Y = 0.64m)
    ((-1.750, 0.000), (0.000, 0.000)),    # Lane 2 (Y = 0.00m)

    # 2. Vertical Connector Lines
    ((-0.400, 0.000), (-0.400, 0.640)),   # Central Switch Line (X = -0.40m)
    ((0.000, -0.640), (0.000, 0.640)),    # Central Distribution Trunk (X = 0.00m)

    # 3. Branch Lines to Drop-Off Zones
    ((0.000, 0.220), (0.550, 0.220)),     # Zone 2 Branch (CPU: Y = 0.22m)
    ((0.000, -0.220), (0.550, -0.220)),   # Zone 3 Branch (QR: Y = -0.22m)
    ((0.000, -0.640), (0.550, -0.640)),   # Zone 4 Branch (Chip: Y = -0.64m)

    # 4. Stop Bars (Cross lines: 150mm length)
    ((-1.650, 0.640 - 0.075), (-1.650, 0.640 + 0.075)),    # Rack 1 Stop
    ((-1.650, 0.000 - 0.075), (-1.650, 0.000 + 0.075)),    # Rack 2 Stop
    ((0.550, 0.640 - 0.075), (0.550, 0.640 + 0.075)),      # Dropoff 1 Stop
    ((0.550, 0.220 - 0.075), (0.550, 0.220 + 0.075)),      # Dropoff 2 Stop
    ((0.550, -0.220 - 0.075), (0.550, -0.220 + 0.075)),    # Dropoff 3 Stop
    ((0.550, -0.640 - 0.075), (0.550, -0.640 + 0.075)),    # Dropoff 4 Stop
]


def dist_point_to_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Computes minimum Euclidean distance from 2D point P to segment AB."""
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1

    c1 = wx * vx + wy * vy
    if c1 <= 0.0:
        return math.hypot(px - x1, py - y1)

    c2 = vx * vx + vy * vy
    if c2 <= c1:
        return math.hypot(px - x2, py - y2)

    t = c1 / c2
    proj_x = x1 + t * vx
    proj_y = y1 + t * vy
    return math.hypot(px - proj_x, py - proj_y)


def min_dist_to_line_network(px: float, py: float) -> float:
    """Finds minimum distance from point P to any line segment on the track."""
    min_d = float('inf')
    for (x1, y1), (x2, y2) in TRACK_SEGMENTS:
        d = dist_point_to_segment(px, py, x1, y1, x2, y2)
        if d < min_d:
            min_d = d
    return min_d


class VectorLineSensorNode(Node):
    def __init__(self):
        super().__init__('vector_line_sensor_node')

        # Parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('enable_rear_array', True)
        self.declare_parameter('num_sensors_front', 8)
        self.declare_parameter('sensor_spacing_front', 0.018)  # 18mm
        self.declare_parameter('offset_x_front', 0.18)         # +180mm
        self.declare_parameter('offset_y_front', 0.0)

        self.declare_parameter('num_sensors_rear', 8)
        self.declare_parameter('sensor_spacing_rear', 0.018)   # 18mm
        self.declare_parameter('offset_x_rear', -0.18)         # -180mm
        self.declare_parameter('offset_y_rear', 0.0)

        self.declare_parameter('line_width', LINE_WIDTH)
        self.declare_parameter('update_rate', 50.0)            # 50 Hz
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'odom')

        # Read Parameters
        self.enable_rear = bool(self.get_parameter('enable_rear_array').value)
        self.n_front = int(self.get_parameter('num_sensors_front').value)
        self.spacing_front = float(self.get_parameter('sensor_spacing_front').value)
        self.offset_x_front = float(self.get_parameter('offset_x_front').value)
        self.offset_y_front = float(self.get_parameter('offset_y_front').value)

        self.n_rear = int(self.get_parameter('num_sensors_rear').value)
        self.spacing_rear = float(self.get_parameter('sensor_spacing_rear').value)
        self.offset_x_rear = float(self.get_parameter('offset_x_rear').value)
        self.offset_y_rear = float(self.get_parameter('offset_y_rear').value)

        self.line_w = float(self.get_parameter('line_width').value)
        self.rate = float(self.get_parameter('update_rate').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.world_frame = self.get_parameter('world_frame').value

        self.baseline_L = abs(self.offset_x_front - self.offset_x_rear)
        if self.baseline_L < 1e-4:
            self.baseline_L = 0.36

        # Sensor eye local offsets (Ordered from Left +Y to Right -Y)
        half_span_front = (self.n_front - 1) * self.spacing_front / 2.0
        self.front_local_y = np.linspace(half_span_front, -half_span_front, self.n_front) + self.offset_y_front
        self.front_local_x = np.full(self.n_front, self.offset_x_front)

        half_span_rear = (self.n_rear - 1) * self.spacing_rear / 2.0
        self.rear_local_y = np.linspace(half_span_rear, -half_span_rear, self.n_rear) + self.offset_y_rear
        self.rear_local_x = np.full(self.n_rear, self.offset_x_rear)

        # Pose cache
        self.latest_pose = None

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.pub_raw = self.create_publisher(Int32MultiArray, '/line_sensor/raw', 10)
        self.pub_analog = self.create_publisher(Float32MultiArray, '/line_sensor/analog', 10)
        self.pub_error = self.create_publisher(Float32, '/line_sensor/error', 10)
        self.pub_detected = self.create_publisher(Bool, '/line_sensor/line_detected', 10)
        self.pub_junction = self.create_publisher(String, '/line_sensor/junction', 10)

        self.pub_front_raw = self.create_publisher(Int32MultiArray, '/line_sensor/front/raw', 10)
        self.pub_front_analog = self.create_publisher(Float32MultiArray, '/line_sensor/front/analog', 10)
        self.pub_front_error = self.create_publisher(Float32, '/line_sensor/front/error', 10)
        self.pub_front_detected = self.create_publisher(Bool, '/line_sensor/front/line_detected', 10)
        self.pub_front_junction = self.create_publisher(String, '/line_sensor/front/junction', 10)

        self.pub_rear_raw = self.create_publisher(Int32MultiArray, '/line_sensor/rear/raw', 10)
        self.pub_rear_analog = self.create_publisher(Float32MultiArray, '/line_sensor/rear/analog', 10)
        self.pub_rear_error = self.create_publisher(Float32, '/line_sensor/rear/error', 10)
        self.pub_rear_detected = self.create_publisher(Bool, '/line_sensor/rear/line_detected', 10)
        self.pub_rear_junction = self.create_publisher(String, '/line_sensor/rear/junction', 10)

        self.pub_lateral_error = self.create_publisher(Float32, '/line_sensor/lateral_error', 10)
        self.pub_heading_error = self.create_publisher(Float32, '/line_sensor/heading_error', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/line_sensor/markers', 10)

        # Latched /arena/map OccupancyGrid Publisher for RViz2 Arena Floor Lines Display
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_map = self.create_publisher(OccupancyGrid, '/arena/map', map_qos)
        self.arena_map_msg = self._build_occupancy_grid()
        self.pub_map.publish(self.arena_map_msg)
        self.map_timer = self.create_timer(1.0, self._publish_map_periodic)

        # Timer loop
        timer_period = 1.0 / max(1.0, self.rate)
        self.timer = self.create_timer(timer_period, self.update_line_sensor)

        self.get_logger().info(
            f'Vector Line Sensor Running: Front={self.n_front} eyes (+{self.offset_x_front*1000:.0f}mm), '
            f'Rear={self.n_rear} eyes ({self.offset_x_rear*1000:.0f}mm), Pure Vector Mode (No images required)'
        )

    def _build_occupancy_grid(self) -> OccupancyGrid:
        """Generates 2D OccupancyGrid raster map of the entire arena for RViz2 display."""
        res = 0.005  # 5mm resolution
        origin_x, origin_y = -2.1, -1.1
        width_m, height_m = 4.2, 2.2
        grid_w = int(width_m / res)
        grid_h = int(height_m / res)

        xs = origin_x + (np.arange(grid_w, dtype=np.float32) + 0.5) * res
        ys = origin_y + (np.arange(grid_h, dtype=np.float32) + 0.5) * res
        grid_x, grid_y = np.meshgrid(xs, ys)  # Shape (grid_h, grid_w)

        # 0 = Free / White background
        grid = np.zeros((grid_h, grid_w), dtype=np.int8)

        # 1. Rasterize Arena Track Lines (Occupied = 100)
        half_w = self.line_w / 2.0
        for (x1, y1), (x2, y2) in TRACK_SEGMENTS:
            vx = x2 - x1
            vy = y2 - y1
            l2 = vx * vx + vy * vy
            if l2 < 1e-6:
                continue
            wx = grid_x - x1
            wy = grid_y - y1
            t = np.clip((wx * vx + wy * vy) / l2, 0.0, 1.0)
            proj_x = x1 + t * vx
            proj_y = y1 + t * vy
            d2 = (grid_x - proj_x) ** 2 + (grid_y - proj_y) ** 2
            grid[d2 <= (half_w ** 2)] = 100

        # 2. Rasterize Arena Outer Perimeter (60 = Dark Grey)
        border_w = 0.015
        mask_outer = (
            (grid_x >= -2.0) & (grid_x <= 2.0) & (grid_y >= -1.0) & (grid_y <= 1.0) &
            ((grid_x <= -2.0 + border_w) | (grid_x >= 2.0 - border_w) |
             (grid_y <= -1.0 + border_w) | (grid_y >= 1.0 - border_w))
        )
        grid[mask_outer] = 60

        # 3. Rasterize START Zone Outline (80 = Outline)
        mask_start = (grid_x >= -0.985 - 0.15) & (grid_x <= -0.985 + 0.15) & (grid_y >= 0.64 - 0.15) & (grid_y <= 0.64 + 0.15)
        mask_start_border = mask_start & (
            (grid_x <= -0.985 - 0.135) | (grid_x >= -0.985 + 0.135) |
            (grid_y <= 0.64 - 0.135) | (grid_y >= 0.64 + 0.135)
        )
        grid[mask_start_border] = 80

        # 4. Rasterize 4 Drop-Off Zones Outlines (85 = Outline)
        drop_zones = [(0.70, 0.64), (0.70, 0.22), (0.70, -0.22), (0.70, -0.64)]
        for zx, zy in drop_zones:
            mask_z = (grid_x >= zx - 0.13) & (grid_x <= zx + 0.13) & (grid_y >= zy - 0.13) & (grid_y <= zy + 0.13)
            mask_z_border = mask_z & (
                (grid_x <= zx - 0.115) | (grid_x >= zx + 0.115) |
                (grid_y <= zy - 0.115) | (grid_y >= zy + 0.115)
            )
            grid[mask_z_border] = 85

        # 5. Rasterize Rack Base Footprints (40 = Light Grey)
        for rx, ry in [(-1.894, 0.640), (-1.894, 0.000)]:
            mask_rack = (grid_x >= rx - 0.075) & (grid_x <= rx + 0.075) & (grid_y >= ry - 0.135) & (grid_y <= ry + 0.135)
            grid[mask_rack] = 40

        msg = OccupancyGrid()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = res
        msg.info.width = grid_w
        msg.info.height = grid_h
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.001
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        return msg

    def _publish_map_periodic(self):
        self.arena_map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(self.arena_map_msg)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (qy_sq := q.y * q.y) - 2.0 * (q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.latest_pose = (p.x, p.y, yaw)

    def _sample_array(self, sensor_x, sensor_y, rx, ry, cos_yaw, sin_yaw, has_pose):
        """Samples sensor array against vector line segments."""
        digital_readings = []
        analog_readings = []
        half_w = self.line_w / 2.0

        for i in range(len(sensor_x)):
            lx = sensor_x[i]
            ly = sensor_y[i]

            is_on_line = 0
            analog_val = 0.0

            if has_pose:
                wx = rx + (lx * cos_yaw - ly * sin_yaw)
                wy = ry + (lx * sin_yaw + ly * cos_yaw)

                d = min_dist_to_line_network(wx, wy)

                if d <= half_w:
                    is_on_line = 1
                    # Smooth Gaussian/distance analog response
                    analog_val = 1.0 - (d / half_w) * 0.4
                elif d <= self.line_w:
                    is_on_line = 0
                    analog_val = max(0.0, 0.6 * (1.0 - (d - half_w) / half_w))
                else:
                    is_on_line = 0
                    analog_val = 0.0

            digital_readings.append(is_on_line)
            analog_readings.append(float(analog_val))

        active_count = sum(digital_readings)
        is_detected = active_count > 0
        error = 0.0
        if is_detected:
            error = float(np.sum(np.array(digital_readings) * sensor_y) / active_count)

        return digital_readings, analog_readings, error, is_detected

    def _classify_junction(self, digital_readings: list) -> str:
        active_count = sum(digital_readings)
        n = len(digital_readings)

        if active_count == 0:
            return 'LOST'
        if active_count >= 5:
            return 'CROSS'

        left_half = sum(digital_readings[:n // 2])
        right_half = sum(digital_readings[n // 2:])

        if left_half >= 3 and right_half == 0:
            return 'T_LEFT'
        if right_half >= 3 and left_half == 0:
            return 'T_RIGHT'

        return 'NONE'

    def update_line_sensor(self):
        has_pose = self.latest_pose is not None
        rx, ry, yaw = self.latest_pose if has_pose else (0.0, 0.0, 0.0)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # 1. Front Array
        f_dig, f_ana, f_err, f_det = self._sample_array(
            self.front_local_x, self.front_local_y, rx, ry, cos_yaw, sin_yaw, has_pose
        )
        f_junction = self._classify_junction(f_dig)

        # 2. Rear Array
        r_dig, r_ana, r_err, r_det = self._sample_array(
            self.rear_local_x, self.rear_local_y, rx, ry, cos_yaw, sin_yaw, has_pose
        )
        r_junction = self._classify_junction(r_dig)

        # 3. Combined Dual Array Errors
        line_detected_any = f_det or r_det
        if f_det and r_det:
            lateral_error = (f_err + r_err) / 2.0
            heading_error = math.atan2(f_err - r_err, self.baseline_L)
        elif f_det:
            lateral_error = f_err
            heading_error = 0.0
        elif r_det:
            lateral_error = r_err
            heading_error = 0.0
        else:
            lateral_error = 0.0
            heading_error = 0.0

        combined_junction = f_junction if f_junction != 'NONE' else r_junction

        # 4. RViz 3D Markers
        marker_array = MarkerArray()
        stamp_now = self.get_clock().now().to_msg()

        # Front Mounting Bar
        bar_f = Marker()
        bar_f.header.frame_id = self.base_frame
        bar_f.header.stamp = stamp_now
        bar_f.ns = 'line_sensor_bars'
        bar_f.id = 100
        bar_f.type = Marker.CUBE
        bar_f.action = Marker.ADD
        bar_f.pose.position.x = self.offset_x_front
        bar_f.pose.position.y = self.offset_y_front
        bar_f.pose.position.z = 0.02
        bar_f.scale.x = 0.015
        bar_f.scale.y = (self.n_front - 1) * self.spacing_front + 0.03
        bar_f.scale.z = 0.008
        bar_f.color.r, bar_f.color.g, bar_f.color.b, bar_f.color.a = 0.15, 0.15, 0.15, 0.95
        marker_array.markers.append(bar_f)

        # Front Sensor Eyes
        for i in range(self.n_front):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = stamp_now
            m.ns = 'front_sensor_dots'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(self.front_local_x[i])
            m.pose.position.y = float(self.front_local_y[i])
            m.pose.position.z = 0.025
            m.scale.x, m.scale.y, m.scale.z = 0.016, 0.016, 0.016

            if f_dig[i] == 1:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0  # Green ON Line
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.4, 0.4, 0.4, 0.5  # Grey OFF Line
            marker_array.markers.append(m)

        # Rear Array Markers
        if self.enable_rear:
            bar_r = Marker()
            bar_r.header.frame_id = self.base_frame
            bar_r.header.stamp = stamp_now
            bar_r.ns = 'line_sensor_bars'
            bar_r.id = 101
            bar_r.type = Marker.CUBE
            bar_r.action = Marker.ADD
            bar_r.pose.position.x = self.offset_x_rear
            bar_r.pose.position.y = self.offset_y_rear
            bar_r.pose.position.z = 0.02
            bar_r.scale.x = 0.015
            bar_r.scale.y = (self.n_rear - 1) * self.spacing_rear + 0.03
            bar_r.scale.z = 0.008
            bar_r.color.r, bar_r.color.g, bar_r.color.b, bar_r.color.a = 0.15, 0.15, 0.15, 0.95
            marker_array.markers.append(bar_r)

            for i in range(self.n_rear):
                m = Marker()
                m.header.frame_id = self.base_frame
                m.header.stamp = stamp_now
                m.ns = 'rear_sensor_dots'
                m.id = 10 + i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(self.rear_local_x[i])
                m.pose.position.y = float(self.rear_local_y[i])
                m.pose.position.z = 0.025
                m.scale.x, m.scale.y, m.scale.z = 0.016, 0.016, 0.016

                if r_dig[i] == 1:
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 1.0, 1.0  # Cyan ON Line
                else:
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.4, 0.4, 0.4, 0.5  # Grey OFF Line
                marker_array.markers.append(m)

        # 5. Deviation Error & Heading Arrows (RViz Visualization)
        # Front Array Lateral Error Arrow (id=200)
        arrow_f = Marker()
        arrow_f.header.frame_id = self.base_frame
        arrow_f.header.stamp = stamp_now
        arrow_f.ns = 'line_error_arrows'
        arrow_f.id = 200
        if f_det:
            arrow_f.type = Marker.ARROW
            arrow_f.action = Marker.ADD
            arrow_f.scale.x = 0.007  # shaft diameter
            arrow_f.scale.y = 0.016  # head diameter
            arrow_f.scale.z = 0.022  # head length
            arrow_f.color.r, arrow_f.color.g, arrow_f.color.b, arrow_f.color.a = 1.0, 0.25, 0.0, 1.0  # Orange-Red
            # Avoid RViz degenerate zero-length arrow warning
            eff_f_y = float(f_err) if abs(f_err) >= 0.003 else (0.003 if f_err >= 0 else -0.003)
            p_start_f = Point(x=float(self.offset_x_front), y=0.0, z=0.038)
            p_end_f = Point(x=float(self.offset_x_front), y=eff_f_y, z=0.038)
            arrow_f.points = [p_start_f, p_end_f]
        else:
            arrow_f.action = Marker.DELETE
        marker_array.markers.append(arrow_f)

        # Rear Array Lateral Error Arrow (id=201)
        if self.enable_rear:
            arrow_r = Marker()
            arrow_r.header.frame_id = self.base_frame
            arrow_r.header.stamp = stamp_now
            arrow_r.ns = 'line_error_arrows'
            arrow_r.id = 201
            if r_det:
                arrow_r.type = Marker.ARROW
                arrow_r.action = Marker.ADD
                arrow_r.scale.x = 0.007
                arrow_r.scale.y = 0.016
                arrow_r.scale.z = 0.022
                arrow_r.color.r, arrow_r.color.g, arrow_r.color.b, arrow_r.color.a = 0.0, 0.85, 1.0, 1.0  # Cyan
                eff_r_y = float(r_err) if abs(r_err) >= 0.003 else (0.003 if r_err >= 0 else -0.003)
                p_start_r = Point(x=float(self.offset_x_rear), y=0.0, z=0.038)
                p_end_r = Point(x=float(self.offset_x_rear), y=eff_r_y, z=0.038)
                arrow_r.points = [p_start_r, p_end_r]
            else:
                arrow_r.action = Marker.DELETE
            marker_array.markers.append(arrow_r)

        # Center Lateral Error Arrow (id=202)
        arrow_c = Marker()
        arrow_c.header.frame_id = self.base_frame
        arrow_c.header.stamp = stamp_now
        arrow_c.ns = 'line_error_arrows'
        arrow_c.id = 202
        if line_detected_any:
            arrow_c.type = Marker.ARROW
            arrow_c.action = Marker.ADD
            arrow_c.scale.x = 0.009
            arrow_c.scale.y = 0.020
            arrow_c.scale.z = 0.028
            arrow_c.color.r, arrow_c.color.g, arrow_c.color.b, arrow_c.color.a = 1.0, 0.9, 0.0, 1.0  # Bright Yellow
            eff_lat_y = float(lateral_error) if abs(lateral_error) >= 0.003 else (0.003 if lateral_error >= 0 else -0.003)
            p_start_c = Point(x=0.0, y=0.0, z=0.045)
            p_end_c = Point(x=0.0, y=eff_lat_y, z=0.045)
            arrow_c.points = [p_start_c, p_end_c]
        else:
            arrow_c.action = Marker.DELETE
        marker_array.markers.append(arrow_c)

        # Track Heading Alignment Arrow (id=203) - Biểu diễn vector hướng vạch line
        arrow_h = Marker()
        arrow_h.header.frame_id = self.base_frame
        arrow_h.header.stamp = stamp_now
        arrow_h.ns = 'line_error_arrows'
        arrow_h.id = 203
        if f_det and r_det:
            arrow_h.type = Marker.ARROW
            arrow_h.action = Marker.ADD
            arrow_h.scale.x = 0.006
            arrow_h.scale.y = 0.014
            arrow_h.scale.z = 0.022
            arrow_h.color.r, arrow_h.color.g, arrow_h.color.b, arrow_h.color.a = 0.2, 1.0, 0.2, 0.9  # Lime Green
            # Vector from rear detected line point to front detected line point extended
            p_r_line = Point(x=float(self.offset_x_rear), y=float(r_err), z=0.035)
            # Extend forward slightly
            ext_x = float(self.offset_x_front) + 0.08
            ext_y = float(f_err) + (float(f_err) - float(r_err)) / self.baseline_L * 0.08
            p_f_line = Point(x=ext_x, y=ext_y, z=0.035)
            arrow_h.points = [p_r_line, p_f_line]
        else:
            arrow_h.action = Marker.DELETE
        marker_array.markers.append(arrow_h)

        # 3D Floating HUD Text Status (id=300)
        hud_text = Marker()
        hud_text.header.frame_id = self.base_frame
        hud_text.header.stamp = stamp_now
        hud_text.ns = 'line_sensor_hud'
        hud_text.id = 300
        hud_text.type = Marker.TEXT_VIEW_FACING
        hud_text.action = Marker.ADD
        hud_text.pose.position.x = 0.0
        hud_text.pose.position.y = 0.0
        hud_text.pose.position.z = 0.22
        hud_text.scale.z = 0.035  # Text height: 35mm
        if line_detected_any:
            hud_text.color.r, hud_text.color.g, hud_text.color.b, hud_text.color.a = 1.0, 1.0, 1.0, 1.0
            hud_text.text = (
                f"e_lat: {lateral_error*1000:+.1f}mm | "
                f"e_yaw: {math.degrees(heading_error):+.1f}° | "
                f"[{combined_junction}]"
            )
        else:
            hud_text.color.r, hud_text.color.g, hud_text.color.b, hud_text.color.a = 1.0, 0.3, 0.3, 1.0
            hud_text.text = "LINE: LOST"
        marker_array.markers.append(hud_text)

        # 6. Publish Topics
        self.pub_raw.publish(Int32MultiArray(data=f_dig))
        self.pub_analog.publish(Float32MultiArray(data=f_ana))
        self.pub_error.publish(Float32(data=float(lateral_error)))
        self.pub_detected.publish(Bool(data=line_detected_any))
        self.pub_junction.publish(String(data=combined_junction))

        self.pub_front_raw.publish(Int32MultiArray(data=f_dig))
        self.pub_front_analog.publish(Float32MultiArray(data=f_ana))
        self.pub_front_error.publish(Float32(data=float(f_err)))
        self.pub_front_detected.publish(Bool(data=f_det))
        self.pub_front_junction.publish(String(data=f_junction))

        self.pub_rear_raw.publish(Int32MultiArray(data=r_dig))
        self.pub_rear_analog.publish(Float32MultiArray(data=r_ana))
        self.pub_rear_error.publish(Float32(data=float(r_err)))
        self.pub_rear_detected.publish(Bool(data=r_det))
        self.pub_rear_junction.publish(String(data=r_junction))

        self.pub_lateral_error.publish(Float32(data=float(lateral_error)))
        self.pub_heading_error.publish(Float32(data=float(heading_error)))
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VectorLineSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
