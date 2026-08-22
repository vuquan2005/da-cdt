#!/usr/bin/env python3
"""
YOLO Object Detection Node for ROS 2.
Subscribes to camera image topic, runs inference with Ultralytics YOLO,
and publishes annotated images, target center points, and JSON detection results.
"""

import json
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import cv2

try:
    from cv_bridge import CvBridge, CvBridgeError
    CV_BRIDGE_AVAILABLE = True
except Exception:
    CV_BRIDGE_AVAILABLE = False

try:
    from ultralytics import YOLO
    import torch
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


def ros_img_to_cv2(msg: Image) -> np.ndarray:
    """Safely convert ROS Image message to OpenCV BGR array (NumPy 1.x & 2.x compatible)."""
    if msg.encoding in ('bgr8', '8UC3'):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    elif msg.encoding == 'rgb8':
        img_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        return cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    elif msg.encoding in ('mono8', '8UC1'):
        img_gray = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        return cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    elif msg.encoding in ('rgba8', 'bgra8'):
        img_rgba = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGR if msg.encoding == 'rgba8' else cv2.COLOR_BGRA2BGR)
    elif CV_BRIDGE_AVAILABLE:
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    else:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def cv2_to_ros_img(cv_img: np.ndarray, header=None) -> Image:
    """Safely convert OpenCV BGR array to ROS Image message."""
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = cv_img.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = cv_img.tobytes()
    return msg



class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Declare parameters
        self.declare_parameter('model_path', 'models/best.pt')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('annotated_image_topic', '/yolo/annotated_image')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', '')  # '' = auto (cuda:0 if available, else cpu)
        self.declare_parameter('target_class', '')  # filter specific class name, '' for all
        self.declare_parameter('enable_tracking', False)  # use model.track() if True

        # Get parameter values
        model_param = self.get_parameter('model_path').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.annotated_image_topic = self.get_parameter('annotated_image_topic').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_thresh = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.device_param = self.get_parameter('device').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.enable_tracking = self.get_parameter('enable_tracking').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.fps = 0.0
        self.last_time = time.time()

        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error(
                "Ultralytics library not found! Please run: pip install ultralytics"
            )
            return

        # Resolve model path
        self.model_path = self._resolve_model_path(model_param)
        self.get_logger().info(f"Loading YOLO model from: {self.model_path}")

        # Determine compute device
        if not self.device_param:
            self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = self.device_param
        self.get_logger().info(f"Using compute device: {self.device}")

        # Load YOLO model
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"Model loaded successfully. Class names: {self.model.names}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.model = None
            return

        # Setup QoS for camera subscriber
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers & Publishers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        self.annotated_image_pub = self.create_publisher(
            Image,
            self.annotated_image_topic,
            10
        )

        self.target_point_pub = self.create_publisher(
            PointStamped,
            '/yolo/target_center',
            10
        )

        self.detections_json_pub = self.create_publisher(
            String,
            '/yolo/detections_json',
            10
        )

        self.get_logger().info(
            f"YOLO Detector Node initialized. Subscribed to: {self.image_topic}, "
            f"Publishing annotated images to: {self.annotated_image_topic}"
        )

    def _resolve_model_path(self, model_param: str) -> str:
        """Find the full path to the model weights file."""
        # 1. Direct path (absolute or relative to current working dir)
        if os.path.isfile(model_param):
            return os.path.abspath(model_param)

        basename = os.path.basename(model_param)

        # 2. Check package share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('robot0_vision')
            candidates = [
                os.path.join(pkg_share, model_param),
                os.path.join(pkg_share, 'models', basename),
                os.path.join(pkg_share, basename),
            ]
            for candidate in candidates:
                if os.path.isfile(candidate):
                    return os.path.abspath(candidate)
        except Exception:
            pass

        # 3. Check workspace source / relative candidates
        candidates = [
            os.path.join(os.getcwd(), 'src', 'robot0_vision', 'models', basename),
            os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src', 'robot0_vision', 'models', basename),
            os.path.join(os.path.dirname(__file__), '..', 'models', basename),
            os.path.join('/workspaces/ros-cdt', 'src', 'robot0_vision', 'models', basename),
        ]
        for candidate in candidates:
            if os.path.isfile(candidate):
                return os.path.abspath(candidate)

        self.get_logger().warn(
            f"Model file '{model_param}' not found in candidate paths. Falling back to 'yolov8n.pt'"
        )
        return 'yolov8n.pt'

    def image_callback(self, msg: Image):
        if self.model is None:
            return

        # Calculate FPS
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if self.fps > 0 else 1.0 / dt
        self.last_time = current_time

        # Convert ROS Image -> OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return

        img_height, img_width = cv_image.shape[:2]

        # Run inference / tracking
        try:
            if self.enable_tracking:
                results = self.model.track(
                    source=cv_image,
                    conf=self.conf_thresh,
                    iou=self.iou_thresh,
                    device=self.device,
                    persist=True,
                    verbose=False
                )
            else:
                results = self.model.predict(
                    source=cv_image,
                    conf=self.conf_thresh,
                    iou=self.iou_thresh,
                    device=self.device,
                    verbose=False
                )
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return

        # Parse detections
        detections = []
        best_target = None
        max_area = 0.0

        if results and len(results) > 0:
            result = results[0]
            # Draw standard YOLO annotations
            annotated_frame = result.plot()

            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    cls_name = self.model.names.get(cls_id, str(cls_id))
                    conf = float(box.conf[0].item())
                    xyxy = box.xyxy[0].tolist()  # [xmin, ymin, xmax, ymax]
                    track_id = int(box.id[0].item()) if (box.id is not None) else None

                    xmin, ymin, xmax, ymax = xyxy
                    box_w = xmax - xmin
                    box_h = ymax - ymin
                    cx = xmin + box_w / 2.0
                    cy = ymin + box_h / 2.0
                    area_ratio = (box_w * box_h) / (img_width * img_height)

                    det_info = {
                        'class_id': cls_id,
                        'class_name': cls_name,
                        'confidence': round(conf, 3),
                        'bbox': [round(v, 2) for v in xyxy],
                        'center': [round(cx, 2), round(cy, 2)],
                        'track_id': track_id
                    }
                    detections.append(det_info)

                    # Filter for target object (largest matching object)
                    if (not self.target_class) or (cls_name.lower() == self.target_class.lower()):
                        if area_ratio > max_area:
                            max_area = area_ratio
                            best_target = {
                                'cx': cx,
                                'cy': cy,
                                'area_ratio': area_ratio,
                                'class_name': cls_name
                            }
        else:
            annotated_frame = cv_image.copy()

        # Publish target center point if found
        if best_target is not None:
            # Normalized coordinates: x: [-1.0 (left) .. 0 (center) .. +1.0 (right)]
            # y: [-1.0 (top) .. 0 (center) .. +1.0 (bottom)]
            norm_x = (best_target['cx'] - img_width / 2.0) / (img_width / 2.0)
            norm_y = (best_target['cy'] - img_height / 2.0) / (img_height / 2.0)

            point_msg = PointStamped()
            point_msg.header = msg.header
            point_msg.header.frame_id = 'camera_optical_link'
            point_msg.point.x = float(norm_x)
            point_msg.point.y = float(norm_y)
            point_msg.point.z = float(best_target['area_ratio'])
            self.target_point_pub.publish(point_msg)

            # Draw target crosshair and label
            t_cx, t_cy = int(best_target['cx']), int(best_target['cy'])
            cv2.drawMarker(
                annotated_frame,
                (t_cx, t_cy),
                (0, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=20,
                thickness=2
            )
            cv2.putText(
                annotated_frame,
                f"TARGET: {best_target['class_name']} dx:{norm_x:+.2f} dy:{norm_y:+.2f}",
                (t_cx - 60, max(20, t_cy - 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        # Draw FPS and summary overlay
        cv2.putText(
            annotated_frame,
            f"FPS: {self.fps:.1f} | Objects: {len(detections)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge publish error: {e}")

        # Publish JSON detections summary
        json_msg = String()
        json_msg.data = json.dumps({
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'count': len(detections),
            'detections': detections
        })
        self.detections_json_pub.publish(json_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
