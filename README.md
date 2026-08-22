# ROS 2 Mecanum Robot Simulation (`ros-cdt`)

Dự án phát triển và mô phỏng robot di động 4 bánh Mecanum tích hợp cơ cấu tay nâng (`robot0`) sử dụng **ROS 2 Humble** và **Gazebo Fortress (GZ Sim)** trong môi trường Dev Container.

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
│   ├── robot0_description/        # 📦 Mô hình CAD, URDF & Hiển thị tĩnh (ament_cmake)
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── urdf/
│   │   │   └── robot0.urdf         # Mô tả robot (khớp bánh Mecanum, tay nâng, sensors, Gazebo plugins)
│   │   ├── meshes/                 # File CAD 3D (.STL) bánh xe, khung sườn, tay nâng
│   │   ├── config/
│   │   │   └── controllers.yaml    # Cấu hình Controller
│   │   ├── rviz/
│   │   │   └── robot0.rviz         # Cấu hình hiển thị RViz2
│   │   └── launch/
│   │       └── display.launch.py   # Kiểm tra TF và khớp trên RViz2 với GUI
│   │
│   ├── robot0_gazebo/             # 🌍 Môi trường mô phỏng Gazebo (ament_cmake)
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   │   └── ros_gz_bridge.yaml  # Cầu nối dữ liệu ROS 2 <-> Gazebo Sim
│   │   ├── models/                 # Mô hình 3D sân đấu và vật thể (aluminum, chip, cpu, qr)
│   │   ├── worlds/
│   │   │   ├── empty.sdf           # Thế giới mô phỏng phẳng cơ bản
│   │   │   └── robocon_arena.sdf   # Sa bàn sân thi đấu Robocon
│   │   └── launch/
│   │       └── gazebo.launch.py    # Khởi chạy Gazebo + Bridge + Robot State Publisher + RViz2
│   │
│   ├── robot0_teleop/             # 🎮 Điều khiển từ xa & Joystick (ament_python)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── config/
│   │   │   └── joy_teleop.yaml     # Cấu hình nút bấm, deadzone, dải nâng tay gắp
│   │   ├── launch/
│   │   │   └── joystick.launch.py  # Khởi chạy joy_node + teleop_node
│   │   └── robot0_teleop/
│   │       └── teleop_node.py      # Xử lý động học Mecanum, auto-brake & điều khiển tay nâng
│   │
│   └── robot0_vision/             # 👁️ Nhận diện ảnh & bám mục tiêu YOLO (ament_python)
│       ├── package.xml
│       ├── setup.py
│       ├── config/
│       │   └── yolo_params.yaml    # Cấu hình model_path, confidence, topic
│       ├── models/
│       │   └── best.pt             # Trọng số mô hình YOLO đã huấn luyện
│       ├── launch/
│       │   └── yolo_detector.launch.py # Khởi chạy node nhận diện YOLO
│       └── robot0_vision/
│           └── yolo_detector_node.py   # Node YOLO inference & visual tracking
│
├── .gitignore                      # Bỏ qua build/, install/, log/, python cache
└── README.md
```

---

## 🛠️ Yêu cầu & Cài đặt môi trường

### Mở bằng VS Code Dev Container (Khuyến nghị)

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

*(Hoặc dùng phím tắt **`Ctrl+Shift+B`** trong VS Code).*

---

### 2. Xem mô hình trên RViz2 (Kiểm tra TF tĩnh)

Kiểm tra cấu trúc cây liên kết TF và điều khiển góc quay khớp bằng giao diện GUI mà không cần nạp mô phỏng vật lý:

```bash
ros2 launch robot0_description display.launch.py
```

---

### 3. Khởi chạy mô phỏng vật lý trên Gazebo Fortress

Khởi động thế giới sa bàn Robocon, nạp robot, kết nối Bridge và mở RViz2:

```bash
ros2 launch robot0_gazebo gazebo.launch.py
```

> **Tùy chọn**:
> * Tắt RViz2 để giảm tải CPU/GPU:
>   ```bash
>   ros2 launch robot0_gazebo gazebo.launch.py rviz:=false
>   ```
> * Chọn thế giới khác (ví dụ `empty.sdf`):
>   ```bash
>   ros2 launch robot0_gazebo gazebo.launch.py world:=$(ros2 pkg prefix robot0_gazebo)/share/robot0_gazebo/worlds/empty.sdf
>   ```

---

### 4. Điều khiển Robot di chuyển & Tay nâng

Mở một terminal mới (trong container) và chọn một trong các phương thức sau:

#### 🕹️ Cách 1: Điều khiển bằng tay cầm Gamepad / Joystick (`robot0_teleop`) - Khuyến nghị

1. **Kiểm tra tay cầm:**
   ```bash
   ls -l /dev/input/js*
   jstest /dev/input/js0    # hoặc js1
   ```

2. **Khởi chạy Launch file:**
   ```bash
   ros2 launch robot0_teleop joystick.launch.py
   ```

3. **Bảng điều khiển (Mapping nút bấm Gamepad):**

   | Phím / Cần gạt | Chức năng | Ghi chú |
   | :--- | :--- | :--- |
   | **`LB / L1`** | **Deadman Switch (Nút an toàn)** | **Phải giữ nút này** khi gạt cần để robot di chuyển. |
   | **`LT / L2`** | **Độ lợi tịnh tiến (Linear Gain)** | Không nhấn: $1.0\times$ / Nhấn nhẹ: $0.5\times$ (chính xác) $\to$ Hết hành trình: $3.0\times$ (tăng tốc). |
   | **`RT / R2`** | **Độ lợi tự xoay (Angular Gain)** | Nhấn cò để tăng độ lợi xoay góc (Yaw) mượt mà từ $1.0\times \to 3.0\times$. |
   | **`RB / R1`** | **Turbo Digital (Tối đa)** | Giữ nút để tăng tức thì cả vận tốc tịnh tiến & xoay lên mức $3.0\times$. |
   | **Cần trái (Left Stick)** | **Tiến / Lùi / Đi ngang (Strafe)** | Điều khiển di chuyển toàn hướng Mecanum. |
   | **Cần phải (Right Stick)** | **Xoay góc (Yaw)** | Xoay trái / phải. |
   | **Nút `Y`** | **Nâng tay gắp (Lift Up)** | Tăng chiều cao tay nâng (tối đa 0.18 m). |
   | **Nút `A`** | **Hạ tay gắp (Lift Down)** | Giảm chiều cao tay nâng (tối thiểu 0.0 m). |

#### 🎮 Cách 2: Điều khiển bằng bàn phím (Keyboard Teleop)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
* `i`: Đi thẳng / `k`: Dừng / `,`: Lùi
* `j`: Xoay trái / `l`: Xoay phải
* `u` / `o`: Đi chéo trái / phải
* `J` / `L` (Shift): Đi ngang trái / phải (Mecanum Strafe)

#### 📡 Cách 3: Gửi lệnh trực tiếp qua Topic

```bash
# Di chuyển tịnh tiến sang ngang (0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {z: 0.0}}" -r 10

# Điều khiển nâng tay gắp lên độ cao 0.12 m
ros2 topic pub /lift_joint_cmd std_msgs/msg/Float64 "{data: 0.12}" -1
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

### 6. Khởi chạy toàn bộ hệ thống đồng thời (Mô phỏng + Joy + Vision)

Có 2 cách để chạy đồng thời cả 3 thành phần:

#### ⚡ Cách 1: Sử dụng VS Code Tasks (Giao diện đồ họa)
1. Nhấn tổ hợp phím **`Ctrl+Shift+P`** (hoặc `F1`) -> gõ **`Tasks: Run Task`**.
2. Chọn một trong hai task:
   * **`ROS 2: Run All (Gazebo + Joy + Vision) [Split Terminals]`**: Tự động mở 3 terminal song song để bạn dễ quan sát log riêng của từng thành phần.
   * **`ROS 2: Launch All (Single Terminal)`**: Chạy toàn bộ trên 1 terminal duy nhất.

#### 🚀 Cách 2: Khởi chạy bằng Master Launch File (Dòng lệnh)
```bash
ros2 launch robot0_gazebo all.launch.py
```
> **Tùy chọn bổ sung:**
> * Tắt bớt thành phần không cần thiết (ví dụ tắt Joy hoặc Vision):
>   ```bash
>   ros2 launch robot0_gazebo all.launch.py joy:=false
>   ros2 launch robot0_gazebo all.launch.py vision:=false rviz:=false
>   ```

---

## 📊 Danh sách Topic chính

| Topic | ROS 2 Message Type | Chức năng |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Vận tốc di chuyển khung gầm robot |
| `/lift_joint_cmd` | `std_msgs/msg/Float64` | Lệnh vị trí độ cao nâng/hạ tay gắp |
| `/wheel_fl_cmd_vel` | `std_msgs/msg/Float64` | Vận tốc quay bánh trước - trái (Mô phỏng) |
| `/wheel_fr_cmd_vel` | `std_msgs/msg/Float64` | Vận tốc quay bánh trước - phải (Mô phỏng) |
| `/wheel_rl_cmd_vel` | `std_msgs/msg/Float64` | Vận tốc quay bánh sau - trái (Mô phỏng) |
| `/wheel_rr_cmd_vel` | `std_msgs/msg/Float64` | Vận tốc quay bánh sau - phải (Mô phỏng) |
| `/odom` | `nav_msgs/msg/Odometry` | Tọa độ và vận tốc thực tế từ mô phỏng |
| `/joint_states` | `sensor_msgs/msg/JointState` | Góc và vận tốc quay của các khớp bánh xe và tay nâng |
| `/clock` | `rosgraph_msgs/msg/Clock` | Đồng bộ thời gian mô phỏng Gazebo |
| `/tf` | `tf2_msgs/msg/TFMessage` | Cây biến đổi hệ tọa độ (`odom` -> `base_footprint` -> `base_link`) |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Luồng hình ảnh RGB từ camera robot |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Thông số nội tại camera (intrinsics) |
| `/yolo/annotated_image` | `sensor_msgs/msg/Image` | Ảnh đã vẽ Bounding Box, nhãn nhận diện & FPS |
| `/yolo/target_center` | `geometry_msgs/msg/PointStamped` | Tọa độ lệch chuẩn hóa $(dx, dy)$ của mục tiêu |
| `/yolo/detections_json` | `std_msgs/msg/String` | Danh sách chi tiết các object nhận diện (JSON) |
