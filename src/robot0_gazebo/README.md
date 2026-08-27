# robot0_gazebo

Package cấu hình môi trường mô phỏng vật lý trên Gazebo Fortress (Ignition), bao gồm sa bàn sân thi đấu (`simple_arena.sdf`), các mô hình vật thể (kệ hàng, pallet), plugin C++ điều khiển chuyển động mặt phẳng `PlanarVelocityControl`, và cấu hình cầu nối dữ liệu `ros_gz_bridge`.

---

## Khởi chạy mô phỏng

### 1. Khởi chạy Gazebo cơ bản

```bash
ros2 launch robot0_gazebo gazebo.launch.py
```
* `rviz:=false`: Tắt mở kèm RViz2.
* `world:=<path>`: Chọn đường dẫn file world khác (mặc định: `simple_arena.sdf`).

### 2. Khởi chạy toàn bộ hệ thống (Bringup)

Khởi chạy đồng thời Gazebo + Bridge + Joy Teleop + Line Sensor + YOLO Vision:

```bash
ros2 launch robot0_gazebo all.launch.py
```
* Tùy chọn tắt bớt node: `joy:=false`, `line_sensor:=false`, `vision:=false`, `rviz:=false`.

---

## Cấu hình cầu nối `ros_gz_bridge`

| ROS 2 Topic | Gazebo Topic | Message Type | Hướng truyền |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS $\to$ Gazebo |
| `/lift_joint_cmd` | `/model/robot0/joint/lift_joint/0/cmd_pos` | `std_msgs/msg/Float64` | ROS $\to$ Gazebo |
| `/odom` | `/model/robot0/odometry` | `nav_msgs/msg/Odometry` | Gazebo $\to$ ROS |
| `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo $\to$ ROS |
| `/camera/image_raw` | `/camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo $\to$ ROS |
| `/camera/camera_info` | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo $\to$ ROS |

---

## Cấu trúc thư mục

```text
robot0_gazebo/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── PlanarVelocityControl.cpp   # Plugin C++ điều khiển chuyển động mặt phẳng
├── config/
│   └── ros_gz_bridge.yaml          # Cấu hình cầu nối ROS 2 <-> Gazebo Sim
├── worlds/
│   ├── simple_arena.sdf            # Sân giản lược: 2 Kệ hàng + 4 Pallet (Nhôm, CPU, QR, Chip)
│   └── empty.sdf                   # Thế giới mặt phẳng cơ bản
├── models/                         # Mô hình SDF (arena_floor, storage_rack, pallet_*)
└── launch/
    ├── gazebo.launch.py            # Launch Gazebo + Bridge + Spawner + RViz2
    └── all.launch.py               # Master launch khởi chạy toàn bộ các module
```
