# robot0_bringup

Top-level Bringup & System Orchestration package for **Robot0**.

Package này chịu trách nhiệm khởi chạy toàn bộ hệ thống Robot0 (Mô phỏng Gazebo, Bộ điều khiển Kinematics, Cảm biến dò line, Điều khiển Joystick, Nhận diện thị giác YOLO, và Nhiệm vụ Autonomous Navigation Behavior Tree) theo đúng chuẩn kiến trúc phân tầng của ROS 2.

---

## 🚀 Cách sử dụng

### 1. Khởi chạy toàn bộ hệ thống (Simulation + Tất cả Node ứng dụng)

```bash
ros2 launch robot0_bringup bringup.launch.py
```

_Hoặc khởi chạy đồng thời cả nhiệm vụ tự hành Behavior Tree:_

```bash
ros2 launch robot0_bringup bringup.launch.py mission:=true
```

### 2. Chỉ khởi chạy các Node ứng dụng (khi Simulation đã chạy trước hoặc chạy trên Robot thật)

```bash
ros2 launch robot0_bringup nodes.launch.py
```

### 3. Tùy chọn tham số khi Launch

| Tham số        | Mặc định | Mô tả                                                   |
| :------------- | :------- | :------------------------------------------------------ |
| `use_sim_time` | `true`   | Sử dụng clock mô phỏng Gazebo                           |
| `rviz`         | `true`   | Mở giao diện RViz2                                      |
| `controller`   | `true`   | Khởi chạy node kinematics base controller               |
| `joy`          | `true`   | Khởi chạy node joystick teleop (`/joy` + `teleop_node`) |
| `vision`       | `true`   | Khởi chạy node YOLO detector (`/camera/image_raw`)      |
| `line_sensor`  | `true`   | Khởi chạy node cảm biến dò line Vector Quad Array       |
| `mission`      | `false`  | Khởi chạy nhiệm vụ Behavior Tree tự hành gắp pallet     |
| `conf`         | `0.5`    | Ngưỡng tin cậy của YOLO                                 |
| `imgsz`        | `640`    | Kích thước ảnh inference của YOLO                       |

---

## 📂 Cấu trúc Launch Files

```
robot0_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
└── launch/
    ├── bringup.launch.py       # Master launch file khởi chạy Gazebo + Nodes
    ├── sim_bringup.launch.py   # Alias cho bringup.launch.py
    ├── all.launch.py           # Alias cho bringup.launch.py
    └── nodes.launch.py         # Launch file gom toàn bộ các node ứng dụng
```
