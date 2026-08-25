# robot0_vision

Package nhận diện hình ảnh sử dụng mô hình YOLOv8, thực thi trên worker thread bất đồng bộ để xử lý luồng camera `/camera/image_raw`, xuất kết quả bounding box và tọa độ lệch tâm chuẩn hóa $(dx, dy)$ phục vụ bám mục tiêu (Visual Tracking).

---

## Khởi chạy

```bash
ros2 launch robot0_vision yolo_detector.launch.py
```
* Tùy biến tham số: `conf:=0.6`, `model_path:=models/best.pt`.

---

## Danh sách Topic

| Topic | Kiểu dữ liệu | Vai trò | Ý nghĩa |
| :--- | :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Subscribed | Luồng ảnh đầu vào từ camera robot |
| `/yolo/annotated_image` | `sensor_msgs/msg/Image` | Published | Ảnh kết quả đã vẽ bounding box và nhãn |
| `/yolo/target_center` | `geometry_msgs/msg/PointStamped` | Published | Tọa độ lệch tâm chuẩn hóa $(dx, dy) \in [-1.0, 1.0]$ |
| `/yolo/detections_json` | `std_msgs/msg/String` | Published | Danh sách đối tượng phát hiện định dạng JSON |

---

## Tham số cấu hình (`config/yolo_params.yaml`)

| Tham số | Kiểu | Mặc định | Mô tả |
| :--- | :--- | :--- | :--- |
| `model_path` | `string` | `models/best.pt` | Đường dẫn file trọng số mô hình |
| `conf_threshold` | `float` | `0.5` | Ngưỡng độ tin cậy phát hiện |
| `imgsz` | `int` | `640` | Kích thước ảnh đầu vào mô hình |
| `device` | `string` | `cuda:0` / `cpu` | Thiết bị tính toán |
| `half` | `bool` | `true` | Sử dụng FP16 trên GPU |
| `image_topic` | `string` | `/camera/image_raw` | Topic ảnh đầu vào |
| `target_class` | `string` | `""` (bất kỳ) | Lọc riêng class cần bám mục tiêu |

---

## Cấu trúc thư mục

```text
robot0_vision/
├── setup.py / setup.cfg
├── package.xml
├── models/
│   └── best.pt                # File trọng số mô hình YOLO đã huấn luyện
├── config/
│   └── yolo_params.yaml       # File cấu hình tham số
├── launch/
│   └── yolo_detector.launch.py# Launch file khởi chạy node
└── robot0_vision/
    └── yolo_detector_node.py  # Node xử lý YOLO inference đa luồng
```
