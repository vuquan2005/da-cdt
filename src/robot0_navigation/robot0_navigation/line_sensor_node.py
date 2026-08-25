#!/usr/bin/env python3
"""
Dual Array Line Sensor Simulator Node for Robot0 (Front & Rear Arrays).

Simulates two N-channel optical/IR reflectance sensor arrays mounted at the
front (+X) and rear (-X) of the robot. Samples ground reflectivity from the arena floor
texture map according to robot's real-time pose (TF / Odom) and computes:
  - Individual Front & Rear readings & errors (e_front, e_rear)
  - Robot lateral offset (d_lateral = (e_front + e_rear) / 2)
  - Robot heading error angle (theta_error = atan2(e_front - e_rear, L))
  - Full RViz2 3D Markers & Arena Floor Map
"""

import os
import math
import cv2
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
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32, Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class LineSensorSimulatorNode(Node):
    def __init__(self):
        super().__init__('line_sensor_simulator_node')

        # ==========================================
        # 1. Parameter Declarations
        # ==========================================
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.declare_parameter('enable_rear_array', True)     # True: Dual Array (Front + Rear), False: Front only

        # Front Array Configuration
        self.declare_parameter('num_sensors_front', 8)
        self.declare_parameter('sensor_spacing_front', 0.018) # 18mm
        self.declare_parameter('offset_x_front', 0.18)        # +180mm from base_link
        self.declare_parameter('offset_y_front', 0.0)

        # Rear Array Configuration
        self.declare_parameter('num_sensors_rear', 8)
        self.declare_parameter('sensor_spacing_rear', 0.018)  # 18mm
        self.declare_parameter('offset_x_rear', -0.18)        # -180mm from base_link
        self.declare_parameter('offset_y_rear', 0.0)

        # Detection & Physics
        self.declare_parameter('line_color_white', False)     # False: dark line on bright floor, True: white on dark
        self.declare_parameter('threshold', 90)               # Grayscale threshold
        self.declare_parameter('noise_probability', 0.0)      # Noise rate
        self.declare_parameter('update_rate', 50.0)           # 50 Hz

        # Arena physical dimensions (meters)
        self.declare_parameter('arena_size_x', 4.0)
        self.declare_parameter('arena_size_y', 2.0)
        self.declare_parameter('arena_origin_x', 0.0)
        self.declare_parameter('arena_origin_y', 0.0)

        # Frame IDs
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('image_path', '')

        # Read parameters
        self.enable_rear = bool(self.get_parameter('enable_rear_array').value)

        self.n_front = int(self.get_parameter('num_sensors_front').value)
        self.spacing_front = float(self.get_parameter('sensor_spacing_front').value)
        self.offset_x_front = float(self.get_parameter('offset_x_front').value)
        self.offset_y_front = float(self.get_parameter('offset_y_front').value)

        self.n_rear = int(self.get_parameter('num_sensors_rear').value)
        self.spacing_rear = float(self.get_parameter('sensor_spacing_rear').value)
        self.offset_x_rear = float(self.get_parameter('offset_x_rear').value)
        self.offset_y_rear = float(self.get_parameter('offset_y_rear').value)

        self.line_color_white = bool(self.get_parameter('line_color_white').value)
        self.threshold = int(self.get_parameter('threshold').value)
        self.noise_prob = float(self.get_parameter('noise_probability').value)
        self.rate = float(self.get_parameter('update_rate').value)

        self.arena_size_x = float(self.get_parameter('arena_size_x').value)
        self.arena_size_y = float(self.get_parameter('arena_size_y').value)
        self.arena_origin_x = float(self.get_parameter('arena_origin_x').value)
        self.arena_origin_y = float(self.get_parameter('arena_origin_y').value)

        self.base_frame = self.get_parameter('base_frame').value
        self.world_frame = self.get_parameter('world_frame').value

        # Inter-array baseline distance along X
        self.baseline_L = abs(self.offset_x_front - self.offset_x_rear)
        if self.baseline_L < 1e-4:
            self.baseline_L = 0.36

        # Calculate local positions (Front: Ordered from Left +Y to Right -Y)
        half_span_front = (self.n_front - 1) * self.spacing_front / 2.0
        self.front_local_y = np.linspace(half_span_front, -half_span_front, self.n_front) + self.offset_y_front
        self.front_local_x = np.full(self.n_front, self.offset_x_front)

        # Calculate local positions (Rear: Ordered from Left +Y to Right -Y)
        half_span_rear = (self.n_rear - 1) * self.spacing_rear / 2.0
        self.rear_local_y = np.linspace(half_span_rear, -half_span_rear, self.n_rear) + self.offset_y_rear
        self.rear_local_x = np.full(self.n_rear, self.offset_x_rear)

        # Pose cache (from Odom or TF)
        self.latest_pose = None

        # ==========================================
        # 2. Load Floor Texture Map
        # ==========================================
        img_path = self.get_parameter('image_path').value
        if not img_path:
            try:
                gazebo_share = get_package_share_directory('robot0_gazebo')
                img_path = os.path.join(gazebo_share, 'models', 'arena_floor', 'materials', 'textures', 'floor.png')
            except Exception:
                img_path = ''

        if not img_path or not os.path.exists(img_path):
            rel_texture = os.path.join('models', 'arena_floor', 'materials', 'textures', 'floor.png')
            candidate_paths = [
                os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'robot0_gazebo', rel_texture)),
                os.path.join(os.getcwd(), 'src', 'robot0_gazebo', rel_texture),
                os.path.join(os.getcwd(), 'robot0_gazebo', rel_texture),
                os.path.join(os.getcwd(), rel_texture),
            ]
            for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
                if prefix:
                    candidate_paths.append(os.path.join(prefix, 'share', 'robot0_gazebo', rel_texture))

            for p in candidate_paths:
                if os.path.exists(p):
                    img_path = p
                    break

        self.floor_img = None
        if img_path and os.path.exists(img_path):
            self.floor_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if self.floor_img is not None:
                self.img_h, self.img_w = self.floor_img.shape
                self.get_logger().info(
                    f'Loaded arena floor texture ({self.img_w}x{self.img_h}) from: {img_path}'
                )

        if self.floor_img is None:
            self.get_logger().warn(
                f'Floor texture not found at "{img_path}". Initializing fallback dummy canvas.'
            )
            self.img_h, self.img_w = 854, 1699
            self.floor_img = np.full((self.img_h, self.img_w), 255, dtype=np.uint8)

        # ==========================================
        # 3. TF Buffer & Subscriptions
        # ==========================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ==========================================
        # 4. Publishers
        # ==========================================
        try:
            d_policy = QoSDurabilityPolicy.TRANSIENT_LOCAL
            r_policy = QoSReliabilityPolicy.RELIABLE
            h_policy = QoSHistoryPolicy.KEEP_LAST
        except Exception:
            d_policy = DurabilityPolicy.TRANSIENT_LOCAL
            r_policy = ReliabilityPolicy.RELIABLE
            h_policy = HistoryPolicy.KEEP_LAST

        latched_qos = QoSProfile(
            durability=d_policy,
            reliability=r_policy,
            history=h_policy,
            depth=1
        )

        # Legacy / Combined Topics (Backward compatible)
        self.pub_raw = self.create_publisher(Int32MultiArray, '/line_sensor/raw', 10)
        self.pub_analog = self.create_publisher(Float32MultiArray, '/line_sensor/analog', 10)
        self.pub_error = self.create_publisher(Float32, '/line_sensor/error', 10)
        self.pub_detected = self.create_publisher(Bool, '/line_sensor/line_detected', 10)

        # Front Array Dedicated Topics
        self.pub_front_raw = self.create_publisher(Int32MultiArray, '/line_sensor/front/raw', 10)
        self.pub_front_analog = self.create_publisher(Float32MultiArray, '/line_sensor/front/analog', 10)
        self.pub_front_error = self.create_publisher(Float32, '/line_sensor/front/error', 10)
        self.pub_front_detected = self.create_publisher(Bool, '/line_sensor/front/line_detected', 10)

        # Rear Array Dedicated Topics
        self.pub_rear_raw = self.create_publisher(Int32MultiArray, '/line_sensor/rear/raw', 10)
        self.pub_rear_analog = self.create_publisher(Float32MultiArray, '/line_sensor/rear/analog', 10)
        self.pub_rear_error = self.create_publisher(Float32, '/line_sensor/rear/error', 10)
        self.pub_rear_detected = self.create_publisher(Bool, '/line_sensor/rear/line_detected', 10)

        # Dual Array Kinematics Outputs (Heading & Lateral Deviation)
        self.pub_lateral_error = self.create_publisher(Float32, '/line_sensor/lateral_error', 10)
        self.pub_heading_error = self.create_publisher(Float32, '/line_sensor/heading_error', 10)

        # Visuals
        self.pub_markers = self.create_publisher(MarkerArray, '/line_sensor/markers', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/arena/map', latched_qos)

        # Initial Map publication
        self.publish_arena_map()

        # Timers
        timer_period = 1.0 / max(1.0, self.rate)
        self.timer = self.create_timer(timer_period, self.update_line_sensor)
        self.map_timer = self.create_timer(2.0, self.publish_arena_map)

        self.get_logger().info(
            f'Dual Array Line Sensor Running: Front={self.n_front} eyes ({self.offset_x_front*1000:+.0f}mm), '
            f'Rear={self.n_rear} eyes ({self.offset_x_rear*1000:+.0f}mm), Baseline={self.baseline_L*1000:.0f}mm'
        )

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.latest_pose = (p.x, p.y, yaw)

    def publish_arena_map(self):
        """Publishes the floor image as an OccupancyGrid on /arena/map so lines are visible in RViz2."""
        if self.floor_img is None:
            return

        grid = OccupancyGrid()
        grid.header.frame_id = self.world_frame
        grid.header.stamp = rclpy.time.Time().to_msg()

        res = self.arena_size_x / float(self.img_w)
        grid.info.resolution = float(res)
        grid.info.width = self.img_w
        grid.info.height = self.img_h

        grid.info.origin.position.x = float(self.arena_origin_x - self.arena_size_x / 2.0)
        grid.info.origin.position.y = float(self.arena_origin_y - self.arena_size_y / 2.0)
        grid.info.origin.position.z = 0.001
        grid.info.origin.orientation.w = 1.0

        flipped = cv2.flip(self.floor_img, 0)
        cost_data = np.zeros_like(flipped, dtype=np.int8)
        if self.line_color_white:
            cost_data[flipped >= self.threshold] = 100
        else:
            cost_data[flipped <= self.threshold] = 100

        grid.data = cost_data.flatten().tolist()
        self.pub_map.publish(grid)

    def _sample_array(self, sensor_x, sensor_y, rx, ry, cos_yaw, sin_yaw, has_pose):
        """Samples an array of sensors and returns (digital_list, analog_list, error, is_detected)."""
        digital_readings = []
        analog_readings = []

        for i in range(len(sensor_x)):
            lx = sensor_x[i]
            ly = sensor_y[i]

            is_on_line = 0
            analog_val = 0.0

            if has_pose:
                wx = rx + (lx * cos_yaw - ly * sin_yaw)
                wy = ry + (lx * sin_yaw + ly * cos_yaw)

                u = int(((wx - self.arena_origin_x) / self.arena_size_x + 0.5) * self.img_w)
                v = int((0.5 - (wy - self.arena_origin_y) / self.arena_size_y) * self.img_h)

                gray_val = 255 if not self.line_color_white else 0
                if 0 <= u < self.img_w and 0 <= v < self.img_h:
                    gray_val = int(self.floor_img[v, u])

                if self.line_color_white:
                    analog_val = float(gray_val) / 255.0
                    is_on_line = 1 if gray_val >= self.threshold else 0
                else:
                    analog_val = 1.0 - (float(gray_val) / 255.0)
                    is_on_line = 1 if gray_val <= self.threshold else 0

                if self.noise_prob > 0.0 and np.random.rand() < self.noise_prob:
                    is_on_line = 1 - is_on_line

            digital_readings.append(is_on_line)
            analog_readings.append(analog_val)

        active_count = sum(digital_readings)
        is_detected = active_count > 0
        error = 0.0
        if is_detected:
            error = float(np.sum(np.array(digital_readings) * sensor_y) / active_count)

        return digital_readings, analog_readings, error, is_detected

    def update_line_sensor(self):
        # 1. Obtain robot pose
        rx, ry, yaw = 0.0, 0.0, 0.0
        has_pose = False

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame, rclpy.time.Time()
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            q = transform.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            has_pose = True
        except Exception:
            if self.latest_pose is not None:
                rx, ry, yaw = self.latest_pose
                has_pose = True

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # 2. Sample Front Array
        f_dig, f_ana, f_err, f_det = self._sample_array(
            self.front_local_x, self.front_local_y, rx, ry, cos_yaw, sin_yaw, has_pose
        )

        # 3. Sample Rear Array (if enabled)
        if self.enable_rear:
            r_dig, r_ana, r_err, r_det = self._sample_array(
                self.rear_local_x, self.rear_local_y, rx, ry, cos_yaw, sin_yaw, has_pose
            )
        else:
            r_dig, r_ana, r_err, r_det = [0] * self.n_rear, [0.0] * self.n_rear, 0.0, False

        # 4. Compute Kinematic Lateral & Heading Errors
        lateral_error = 0.0
        heading_error = 0.0

        if f_det and r_det:
            # Both arrays see the line: Full pose estimation
            heading_error = math.atan2(f_err - r_err, self.baseline_L)
            lateral_error = (f_err + r_err) / 2.0
        elif f_det:
            # Only front array sees the line
            lateral_error = f_err
            heading_error = 0.0
        elif r_det:
            # Only rear array sees the line
            lateral_error = r_err
            heading_error = 0.0

        line_detected_any = f_det or r_det

        # 5. Build 3D Markers for RViz2
        stamp_zero = rclpy.time.Time().to_msg()
        marker_array = MarkerArray()

        # Front Mounting Bar
        bar_f = Marker()
        bar_f.header.frame_id = self.base_frame
        bar_f.header.stamp = stamp_zero
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
        bar_f.color.r = 0.1
        bar_f.color.g = 0.1
        bar_f.color.b = 0.1
        bar_f.color.a = 0.9
        marker_array.markers.append(bar_f)

        # Front Sensors Dots (IDs 0..N-1)
        for i in range(self.n_front):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = stamp_zero
            m.ns = 'front_sensor_dots'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(self.front_local_x[i])
            m.pose.position.y = float(self.front_local_y[i])
            m.pose.position.z = 0.025
            m.scale.x = 0.016
            m.scale.y = 0.016
            m.scale.z = 0.016

            if f_dig[i] == 1:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0  # Green
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.4, 0.4, 0.4, 0.5  # Grey

            marker_array.markers.append(m)

        # Front Error Arrow
        if f_det:
            arr_f = Marker()
            arr_f.header.frame_id = self.base_frame
            arr_f.header.stamp = stamp_zero
            arr_f.ns = 'front_error_arrow'
            arr_f.id = 200
            arr_f.type = Marker.ARROW
            arr_f.action = Marker.ADD
            arr_f.points = [
                Point(x=self.offset_x_front, y=self.offset_y_front, z=0.03),
                Point(x=self.offset_x_front, y=float(f_err), z=0.03)
            ]
            arr_f.scale.x, arr_f.scale.y, arr_f.scale.z = 0.006, 0.012, 0.012
            arr_f.color.r, arr_f.color.g, arr_f.color.b, arr_f.color.a = 1.0, 0.8, 0.0, 0.95  # Yellow
            marker_array.markers.append(arr_f)

        # Rear Array Markers (if enabled)
        if self.enable_rear:
            # Rear Mounting Bar
            bar_r = Marker()
            bar_r.header.frame_id = self.base_frame
            bar_r.header.stamp = stamp_zero
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
            bar_r.color.r = 0.1
            bar_r.color.g = 0.1
            bar_r.color.b = 0.1
            bar_r.color.a = 0.9
            marker_array.markers.append(bar_r)

            # Rear Sensors Dots (IDs 10..10+N-1)
            for i in range(self.n_rear):
                m = Marker()
                m.header.frame_id = self.base_frame
                m.header.stamp = stamp_zero
                m.ns = 'rear_sensor_dots'
                m.id = 10 + i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(self.rear_local_x[i])
                m.pose.position.y = float(self.rear_local_y[i])
                m.pose.position.z = 0.025
                m.scale.x = 0.016
                m.scale.y = 0.016
                m.scale.z = 0.016

                if r_dig[i] == 1:
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 1.0, 1.0  # Cyan
                else:
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.4, 0.4, 0.4, 0.5  # Grey

                marker_array.markers.append(m)

            # Rear Error Arrow
            if r_det:
                arr_r = Marker()
                arr_r.header.frame_id = self.base_frame
                arr_r.header.stamp = stamp_zero
                arr_r.ns = 'rear_error_arrow'
                arr_r.id = 201
                arr_r.type = Marker.ARROW
                arr_r.action = Marker.ADD
                arr_r.points = [
                    Point(x=self.offset_x_rear, y=self.offset_y_rear, z=0.03),
                    Point(x=self.offset_x_rear, y=float(r_err), z=0.03)
                ]
                arr_r.scale.x, arr_r.scale.y, arr_r.scale.z = 0.006, 0.012, 0.012
                arr_r.color.r, arr_r.color.g, arr_r.color.b, arr_r.color.a = 0.0, 0.8, 1.0, 0.95  # Cyan
                marker_array.markers.append(arr_r)

            # Center Heading Line between Front & Rear line centers (if both detected)
            if f_det and r_det:
                line_conn = Marker()
                line_conn.header.frame_id = self.base_frame
                line_conn.header.stamp = stamp_zero
                line_conn.ns = 'heading_line'
                line_conn.id = 202
                line_conn.type = Marker.LINE_STRIP
                line_conn.action = Marker.ADD
                line_conn.points = [
                    Point(x=self.offset_x_rear, y=float(r_err), z=0.032),
                    Point(x=self.offset_x_front, y=float(f_err), z=0.032)
                ]
                line_conn.scale.x = 0.008  # line width
                line_conn.color.r, line_conn.color.g, line_conn.color.b, line_conn.color.a = 1.0, 0.2, 0.8, 0.9  # Magenta
                marker_array.markers.append(line_conn)

        # 6. Publish All Messages
        # Legacy
        self.pub_raw.publish(Int32MultiArray(data=f_dig))
        self.pub_analog.publish(Float32MultiArray(data=f_ana))
        self.pub_error.publish(Float32(data=float(lateral_error)))
        self.pub_detected.publish(Bool(data=line_detected_any))

        # Front
        self.pub_front_raw.publish(Int32MultiArray(data=f_dig))
        self.pub_front_analog.publish(Float32MultiArray(data=f_ana))
        self.pub_front_error.publish(Float32(data=float(f_err)))
        self.pub_front_detected.publish(Bool(data=f_det))

        # Rear
        self.pub_rear_raw.publish(Int32MultiArray(data=r_dig))
        self.pub_rear_analog.publish(Float32MultiArray(data=r_ana))
        self.pub_rear_error.publish(Float32(data=float(r_err)))
        self.pub_rear_detected.publish(Bool(data=r_det))

        # Kinematic Errors
        self.pub_lateral_error.publish(Float32(data=float(lateral_error)))
        self.pub_heading_error.publish(Float32(data=float(heading_error)))

        # Markers
        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LineSensorSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
