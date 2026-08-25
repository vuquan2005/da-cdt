#!/usr/bin/env python3
"""
High-Performance YOLO Object Detection & Tracking Node for ROS 2.
Features:
- Decoupled asynchronous worker thread with drop-old-frames queue (Zero-Lag guarantee).
- Fast direct NumPy image conversion (no CvBridge copy overhead).
- Configurable inference resolution (imgsz), precision (FP16 half), and target classes.
- Model warmup at startup to eliminate first-frame latency spike.
- Lazy annotation rendering (skips rendering when no subscribers are listening).
- Real-time performance metrics (FPS, inference time, queue drops).
"""

import json
import os
import threading
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
    """Fast, safe conversion from ROS Image message to OpenCV BGR numpy array."""
    enc = msg.encoding.lower()
    if enc in ('bgr8', '8uc3'):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    elif enc == 'rgb8':
        img_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        return cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    elif enc in ('mono8', '8uc1'):
        img_gray = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        return cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    elif enc == 'rgba8':
        img_rgba = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGR)
    elif enc == 'bgra8':
        img_bgra = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        return cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2BGR)
    elif CV_BRIDGE_AVAILABLE:
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    else:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def cv2_to_ros_img(cv_img: np.ndarray, header=None) -> Image:
    """Fast conversion from OpenCV BGR numpy array to ROS Image message."""
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
        self.declare_parameter('target_point_topic', '/yolo/target_center')
        self.declare_parameter('detections_json_topic', '/yolo/detections_json')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '')  # '' = auto (cuda:0 if available, else cpu)
        self.declare_parameter('half', True)  # FP16 half precision on GPU
        self.declare_parameter('target_class', '')  # Filter specific class name, '' for all
        self.declare_parameter('enable_tracking', False)  # Use ByteTrack tracking
        self.declare_parameter('max_fps', 0.0)  # 0.0 = unlimited, >0 limits FPS
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('qos_reliability', 'best_effort')  # 'best_effort' or 'reliable'

        # Get parameter values
        model_param = self.get_parameter('model_path').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.annotated_image_topic = self.get_parameter('annotated_image_topic').get_parameter_value().string_value
        self.target_point_topic = self.get_parameter('target_point_topic').get_parameter_value().string_value
        self.detections_json_topic = self.get_parameter('detections_json_topic').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_thresh = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.imgsz = self.get_parameter('imgsz').get_parameter_value().integer_value
        self.device_param = self.get_parameter('device').get_parameter_value().string_value
        self.use_half = self.get_parameter('half').get_parameter_value().bool_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.enable_tracking = self.get_parameter('enable_tracking').get_parameter_value().bool_value
        self.max_fps = self.get_parameter('max_fps').get_parameter_value().double_value
        self.publish_annotated_image = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        qos_rel_param = self.get_parameter('qos_reliability').get_parameter_value().string_value.lower()

        # Performance & Timing state
        self.fps = 0.0
        self.inference_time_ms = 0.0
        self.last_time = time.time()
        self._last_inference_time = 0.0
        self._dropped_frames = 0
        self._processed_frames = 0

        # Thread synchronization state
        self._running = True
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._latest_msg = None

        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error(
                "Ultralytics library not found! Please run: pip install ultralytics"
            )
            self.model = None
            return

        # Resolve model path
        self.model_path = self._resolve_model_path(model_param)
        self.get_logger().info(f"Loading YOLO model from: {self.model_path}")

        # Determine compute device
        if not self.device_param:
            self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = self.device_param
        self.get_logger().info(f"Using compute device: {self.device} (FP16 half: {self.use_half and 'cuda' in self.device})")

        # Load YOLO model
        try:
            self.model = YOLO(self.model_path)
            if 'cuda' in str(self.device) and self.use_half:
                try:
                    if hasattr(self.model, 'model') and self.model.model is not None:
                        self.model.model.half().to(self.device)
                except Exception as e:
                    self.get_logger().warn(f"Could not convert model to FP16 half: {e}")
            self.get_logger().info(f"Model loaded successfully. Class names: {self.model.names}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.model = None
            return

        # Pre-resolve target class IDs for fast tensor-level filtering
        self.target_class_ids = None
        if self.target_class:
            matched_ids = [k for k, v in self.model.names.items() if v.lower() == self.target_class.lower()]
            if matched_ids:
                self.target_class_ids = matched_ids
                self.get_logger().info(f"Filtering target class '{self.target_class}' (ID: {matched_ids})")
            else:
                self.get_logger().warn(
                    f"Target class '{self.target_class}' not found in model classes: {self.model.names}"
                )

        # Warm up YOLO model to prevent first-frame latency spike
        self._warmup_model()

        # Setup QoS for camera subscriber
        rel_policy = (
            ReliabilityPolicy.BEST_EFFORT
            if qos_rel_param in ('best_effort', 'sensor_data')
            else ReliabilityPolicy.RELIABLE
        )
        sensor_qos = QoSProfile(
            reliability=rel_policy,
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
            1
        )

        self.target_point_pub = self.create_publisher(
            PointStamped,
            self.target_point_topic,
            10
        )

        self.detections_json_pub = self.create_publisher(
            String,
            self.detections_json_topic,
            10
        )

        # Start decoupled inference worker thread
        self._worker_thread = threading.Thread(
            target=self._inference_worker,
            name="YOLO_Inference_Worker",
            daemon=True
        )
        self._worker_thread.start()

        self.get_logger().info(
            f"YOLO Detector Node initialized (Zero-Lag Worker Thread active).\n"
            f"  Subscribed to: {self.image_topic} [QoS: {qos_rel_param}]\n"
            f"  Inference Size: {self.imgsz}x{self.imgsz} | Max FPS: {self.max_fps if self.max_fps > 0 else 'unlimited'}\n"
            f"  Publishing: {self.annotated_image_topic}, {self.target_point_topic}, {self.detections_json_topic}"
        )

    def _warmup_model(self):
        """Warm up PyTorch CUDA kernels/JIT compilation with a dummy image."""
        try:
            dummy_img = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self.model.predict(
                source=dummy_img,
                imgsz=self.imgsz,
                conf=self.conf_thresh,
                device=self.device,
                verbose=False
            )
            self.get_logger().info("YOLO model warmup completed.")
        except Exception as e:
            self.get_logger().warn(f"Model warmup skipped/failed: {e}")

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
            os.path.join(os.getcwd(), 'models', basename),
            os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src', 'robot0_vision', 'models', basename)),
            os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'models', basename)),
            os.path.abspath(os.path.join(os.path.dirname(__file__), 'models', basename)),
        ]
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
            if prefix:
                candidates.append(os.path.join(prefix, 'share', 'robot0_vision', 'models', basename))
                candidates.append(os.path.join(prefix, 'share', 'robot0_vision', basename))

        for candidate in candidates:
            if os.path.isfile(candidate):
                return os.path.abspath(candidate)

        self.get_logger().warn(
            f"Model file '{model_param}' not found in candidate paths. Falling back to 'yolov8n.pt'"
        )
        return 'yolov8n.pt'

    def image_callback(self, msg: Image):
        """
        Fast, non-blocking subscriber callback.
        Stores the newest frame and signals the worker thread.
        Automatically drops unconsumed older frames to guarantee zero latency buildup.
        """
        if not self._running:
            return

        with self._frame_lock:
            if self._latest_msg is not None:
                self._dropped_frames += 1
            self._latest_msg = msg
            self._frame_event.set()

    def _inference_worker(self):
        """Asynchronous worker thread continuously consuming the latest image frame."""
        while self._running and rclpy.ok():
            # Wait for frame notification with timeout to check running state
            if not self._frame_event.wait(timeout=0.1):
                continue

            with self._frame_lock:
                msg = self._latest_msg
                self._latest_msg = None
                self._frame_event.clear()

            if msg is None or not self._running:
                continue

            # Optional FPS throttling
            if self.max_fps > 0:
                elapsed = time.time() - self._last_inference_time
                min_interval = 1.0 / self.max_fps
                if elapsed < min_interval:
                    time.sleep(min_interval - elapsed)

            self._process_image(msg)
            self._last_inference_time = time.time()
            self._processed_frames += 1

    def _process_image(self, msg: Image):
        """Perform decoding, YOLO inference, coordinate transformation, and publishing."""
        if self.model is None:
            return

        # Decode ROS Image -> OpenCV BGR array
        try:
            cv_image = ros_img_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        img_height, img_width = cv_image.shape[:2]

        # Calculate Pipeline FPS
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0:
            instant_fps = 1.0 / dt
            self.fps = 0.9 * self.fps + 0.1 * instant_fps if self.fps > 0 else instant_fps
        self.last_time = current_time

        # Run inference / tracking
        t_start = time.perf_counter()
        try:
            predict_kwargs = {
                'source': cv_image,
                'conf': self.conf_thresh,
                'iou': self.iou_thresh,
                'device': self.device,
                'imgsz': self.imgsz,
                'verbose': False
            }
            if self.target_class_ids is not None:
                predict_kwargs['classes'] = self.target_class_ids

            if self.enable_tracking:
                predict_kwargs['persist'] = True
                results = self.model.track(**predict_kwargs)
            else:
                results = self.model.predict(**predict_kwargs)
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return

        self.inference_time_ms = (time.perf_counter() - t_start) * 1000.0

        # Parse detections
        detections = []
        best_target = None
        max_area = 0.0

        if results and len(results) > 0:
            result = results[0]
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

        # 1. Publish target center point if found
        norm_x = 0.0
        norm_y = 0.0
        if best_target is not None:
            norm_x = (best_target['cx'] - img_width / 2.0) / (img_width / 2.0)
            norm_y = (best_target['cy'] - img_height / 2.0) / (img_height / 2.0)

            point_msg = PointStamped()
            point_msg.header = msg.header
            point_msg.header.frame_id = 'camera_optical_link'
            point_msg.point.x = float(norm_x)
            point_msg.point.y = float(norm_y)
            point_msg.point.z = float(best_target['area_ratio'])
            self.target_point_pub.publish(point_msg)

        # 2. Publish JSON detections summary
        json_msg = String()
        json_msg.data = json.dumps({
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'fps': round(self.fps, 1),
            'inference_ms': round(self.inference_time_ms, 2),
            'dropped_frames': self._dropped_frames,
            'processed_frames': self._processed_frames,
            'count': len(detections),
            'detections': detections
        })
        self.detections_json_pub.publish(json_msg)

        # 3. Publish annotated image (Lazy evaluation: skip rendering if no subscribers listening)
        if self.publish_annotated_image and self.annotated_image_pub.get_subscription_count() > 0:
            if results and len(results) > 0:
                annotated_frame = results[0].plot()
            else:
                annotated_frame = cv_image.copy()

            # Draw target crosshair and label
            if best_target is not None:
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
                    (max(10, t_cx - 60), max(25, t_cy - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

            # Draw FPS, Inference time, and Queue Drop overlay
            cv2.putText(
                annotated_frame,
                f"FPS: {self.fps:.1f} | Inf: {self.inference_time_ms:.1f}ms | Drop: {self._dropped_frames} | Objs: {len(detections)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

            # Publish annotated image
            try:
                annotated_msg = cv2_to_ros_img(annotated_frame, header=msg.header)
                self.annotated_image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Publish annotated image error: {e}")

    def destroy_node(self):
        """Clean shutdown of background worker thread and ROS node."""
        self._running = False
        self._frame_event.set()
        if hasattr(self, '_worker_thread') and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

