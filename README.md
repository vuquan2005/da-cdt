# ROS 2 Mecanum Robot Simulation (`ros-cdt`)

Dự án phát triển và mô phỏng robot di động 4 bánh Mecanum (`robot0`) sử dụng **ROS 2 Humble** và **Gazebo Fortress (GZ Sim)** trong môi trường Dev Container.

---

## 📁 Cấu trúc dự án

```text
ros-cdt/
├── .devcontainer/                  # Cấu hình môi trường Docker Dev Container
│   ├── Dockerfile                  # ROS 2 Humble + Gazebo Fortress + NVIDIA GPU + Joystick
│   └── devcontainer.json
├── .vscode/                        # Cấu hình VS Code (tasks, settings, extensions)
│   ├── settings.json               # Cấu hình đường dẫn Python/ROS và file associations
│   ├── tasks.json                  # Phím tắt build (Ctrl+Shift+B)
│   └── extensions.json             # Khuyến nghị extension ROS, C++, Python
├── src/
│   ├── robot0_description/        # Package mô tả & mô phỏng robot
│   │   ├── CMakeLists.txt          # ament_cmake build script
│   │   ├── package.xml             # ROS 2 package manifest (format 3)
│   │   ├── urdf/
│   │   │   └── robot0.urdf         # Mô tả robot (khớp bánh xe Mecanum, plugins Gazebo)
│   │   ├── meshes/                 # 30 file CAD 3D (.STL)
│   │   ├── worlds/
│   │   │   └── empty.sdf           # Thế giới mô phỏng Gazebo (Physics, Sun, Ground)
│   │   ├── config/
│   │   │   ├── joystick.yaml       # Cấu hình nút bấm & chế độ Mecanum Holonomic
│   │   │   └── ros_gz_bridge.yaml  # Cầu nối dữ liệu ROS 2 <-> Gazebo
│   │   ├── rviz/
│   │   │   └── robot0.rviz         # Cấu hình hiển thị RViz2
│   │   └── launch/
│   │       ├── display.launch.py   # Mở hiển thị trên RViz2 với GUI chỉnh khớp
│   │       ├── gazebo.launch.py    # Khởi chạy toàn bộ mô phỏng Gazebo + Bridge + RViz2
│   │       └── joystick.launch.py  # Khởi chạy điều khiển bằng tay cầm Joystick
│   └── robot0_vision/             # Package nhận diện ảnh YOLO (Python)
│       ├── package.xml             # ament_python manifest
│       ├── setup.py                # Python package build script
│       ├── config/
│       │   └── yolo_params.yaml    # Cấu hình model_path, confidence, topic
│       ├── models/
│       │   └── best.pt             # Trọng số mô hình YOLO đã huấn luyện
│       ├── launch/
│       │   └── yolo_detector.launch.py # Khởi chạy node nhận diện YOLO
│       └── robot0_vision/
│           └── yolo_detector_node.py   # Node YOLO inference & visual tracking
├── .gitignore                      # Bỏ qua build/, install/, log/, python cache
└── README.md
```

---

## 🛠️ Yêu cầu & Cài đặt môi trường

### 1. Mở bằng VS Code Dev Container (Khuyến nghị)

1. Cài đặt **Docker** và **VS Code** kèm extension **Dev Containers**.
2. Cấp quyền hiển thị GUI X11 trên máy Host (nếu dùng Linux):
   ```bash
   xhost +local:root
   ```
3. Mở thư mục dự án trong VS Code -> Chọn **Reopen in Container** (hoặc `Ctrl+Shift+P` -> `Dev Containers: Reopen in Container`).

---

## 🚀 Hướng dẫn sử dụng

### 1. Build Workspace

Trong terminal container:

```bash
cd /workspaces/ros-cdt
colcon build --symlink-install
source install/setup.bash
```

_(Hoặc dùng phím tắt **`Ctrl+Shift+B`** trong VS Code)._

---

### 2. Xem mô hình trên RViz2 (Không cần mở Gazebo)

Kiểm tra cấu trúc cây liên kết TF và điều khiển góc quay khớp bằng giao diện GUI:

```bash
ros2 launch robot0_description display.launch.py
```

---

### 3. Khởi chạy mô phỏng vật lý trên Gazebo Fortress

Khởi động thế giới mô phỏng, nạp robot, kết nối bridge và mở RViz2:

```bash
ros2 launch robot0_description gazebo.launch.py
```

> **Tùy chọn**: Nếu muốn tắt RViz2 để giảm tải:
>
> ```bash
> ros2 launch robot0_description gazebo.launch.py rviz:=false
> ```

---

### 4. Điều khiển Robot di chuyển (`/cmd_vel`)

Mở một terminal mới (trong container) và chọn một trong các cách sau:

#### 🎮 Cách 1: Điều khiển bằng bàn phím (Keyboard Teleop)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- `i`: Đi thẳng / `k`: Dừng / `,`: Lùi
- `j`: Xoay trái / `l`: Xoay phải
- `u` / `o`: Đi chéo trái / phải
- `J` / `L` (Shift): Đi ngang trái / phải (đặc trưng bánh Mecanum)

#### 🕹️ Cách 2: Điều khiển bằng tay cầm (Joystick / Gamepad)

> **Lưu ý với `keyd` hoặc thiết bị ảo:** `joy_node` trên ROS 2 Humble dùng SDL2; index SDL không nhất thiết trùng `/dev/input/jsN`. Cấu hình mặc định chọn tay cầm theo `device_name`, nên không bị ảnh hưởng khi `keyd` làm thay đổi `js0`/`js1`.

1. **Kiểm tra thiết bị tay cầm:**
   ```bash
   # Liệt kê các cổng joystick hiện có
   ls -l /dev/input/js*

   # Kiểm tra tín hiệu nút bấm (thử js1 hoặc js0)
   jstest /dev/input/js1
   ```

2. **Cách 2.1 - Khởi chạy nhanh bằng Launch file (Khuyến nghị):**
   ```bash
   # Dùng tên tay cầm cấu hình trong config/joystick.yaml
   ros2 launch robot0_description joystick.launch.py
   ```

3. **Cách 2.2 - Khởi chạy từng node thủ công:**
   ```bash
   # Terminal 1: Chạy joy_node; xem tên SDL bằng `ros2 run joy joy_enumerate_devices`
   ros2 run joy joy_node --ros-args -p device_name:="PowerA Xbox Series X Controller"

   # Terminal 2: Chạy teleop_node hỗ trợ di chuyển toàn hướng Mecanum (Holonomic)
   ros2 run teleop_twist_joy teleop_node --ros-args --params-file src/robot0_description/config/joystick.yaml
   ```

   * **Cách điều khiển (Mapping mặc định):**
     * **Giữ nút `LB / L1` (Enable button)** khi gạt cần để cho phép phát lệnh di chuyển an toàn.
     * **Giữ thêm nút `RB / R1` (Turbo)** để di chuyển với tốc độ tối đa.
     * **Cần gạt trái (Left Stick):** Tiến / Lùi / Đi ngang trái - phải (Mecanum Strafe).
     * **Cần gạt phải (Right Stick):** Xoay góc trái / phải (Yaw).


#### 📡 Cách 3: Gửi lệnh trực tiếp qua Topic

```bash
# Đi thẳng về phía trước (0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10

# Di chuyển tịnh tiến sang ngang (0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {z: 0.0}}" -r 10

# Xoay tại chỗ (1.0 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 1.0}}" -r 10
```

---

### 5. Khởi chạy Nhận diện hình ảnh YOLO (`robot0_vision`)

Sau khi đã khởi chạy mô phỏng Gazebo, mở một terminal mới và chạy:

```bash
# Khởi chạy node YOLO với trọng số models/best.pt mặc định
ros2 launch robot0_vision yolo_detector.launch.py

# Hoặc tùy chỉnh ngưỡng confidence hoặc đường dẫn model khác:
ros2 launch robot0_vision yolo_detector.launch.py conf:=0.6 model_path:=models/best.pt
```

Hình ảnh nhận diện cùng Bounding Box sẽ được hiển thị trực tiếp trên RViz2 tại mục **YOLO Detections** (`/yolo/annotated_image`).

---

## 📊 Danh sách Topic chính

| Topic                  | ROS 2 Type                   | Chức năng                                                          |
| ---------------------- | ---------------------------- | ------------------------------------------------------------------ |
| `/cmd_vel`             | `geometry_msgs/msg/Twist`    | Gửi vận tốc điều khiển robot                                       |
| `/odom`                | `nav_msgs/msg/Odometry`      | Tọa độ và vận tốc thực tế từ mô phỏng                              |
| `/joint_states`        | `sensor_msgs/msg/JointState` | Góc và vận tốc quay 4 bánh xe                                      |
| `/clock`               | `rosgraph_msgs/msg/Clock`    | Đồng bộ thời gian mô phỏng Gazebo                                  |
| `/tf`                  | `tf2_msgs/msg/TFMessage`     | Cây biến đổi hệ tọa độ (`odom` -> `base_footprint` -> `base_link`) |
| `/camera/image_raw`    | `sensor_msgs/msg/Image`      | Luồng hình ảnh RGB từ camera robot                                 |
| `/camera/camera_info`  | `sensor_msgs/msg/CameraInfo` | Thông số nội tại camera (intrinsics)                               |
| `/yolo/annotated_image`| `sensor_msgs/msg/Image`      | Ảnh đã vẽ Bounding Box, nhãn nhận diện & FPS                       |
| `/yolo/target_center`  | `geometry_msgs/msg/PointStamped` | Độ lệch chuẩn hóa $(dx, dy)$ của mục tiêu để bám đuổi          |
| `/yolo/detections_json`| `std_msgs/msg/String`        | Danh sách chi tiết các object nhận diện (định dạng JSON)           |

