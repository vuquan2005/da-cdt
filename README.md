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
│   └── robot0_description/        # Package mô tả & mô phỏng robot
│       ├── CMakeLists.txt          # ament_cmake build script
│       ├── package.xml             # ROS 2 package manifest (format 3)
│       ├── urdf/
│       │   └── robot0.urdf         # Mô tả robot (khớp bánh xe Mecanum, plugins Gazebo)
│       ├── meshes/                 # 30 file CAD 3D (.STL)
│       ├── worlds/
│       │   └── empty.sdf           # Thế giới mô phỏng Gazebo (Physics, Sun, Ground)
│       ├── config/
│       │   └── ros_gz_bridge.yaml  # Cầu nối dữ liệu ROS 2 <-> Gazebo
│       ├── rviz/
│       │   └── robot0.rviz         # Cấu hình hiển thị RViz2
│       └── launch/
│           ├── display.launch.py   # Mở hiển thị trên RViz2 với GUI chỉnh khớp
│           └── gazebo.launch.py    # Khởi chạy toàn bộ mô phỏng Gazebo + Bridge + RViz2
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

```bash
# Kiểm tra tay cầm
jstest /dev/input/js0

# Chạy node đọc tay cầm
ros2 run teleop_twist_joy teleop_node
```

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

## 📊 Danh sách Topic chính

| Topic           | ROS 2 Type                   | Chức năng                                                          |
| --------------- | ---------------------------- | ------------------------------------------------------------------ |
| `/cmd_vel`      | `geometry_msgs/msg/Twist`    | Gửi vận tốc điều khiển robot                                       |
| `/odom`         | `nav_msgs/msg/Odometry`      | Tọa độ và vận tốc thực tế từ mô phỏng                              |
| `/joint_states` | `sensor_msgs/msg/JointState` | Góc và vận tốc quay 4 bánh xe                                      |
| `/clock`        | `rosgraph_msgs/msg/Clock`    | Đồng bộ thời gian mô phỏng Gazebo                                  |
| `/tf`           | `tf2_msgs/msg/TFMessage`     | Cây biến đổi hệ tọa độ (`odom` -> `base_footprint` -> `base_link`) |
