# Môi trường ROS 2 Humble + Gazebo Fortress

Mở thư mục dự án bằng VS Code, cài extension **Dev Containers**, rồi chọn
**Dev Containers: Reopen in Container**. Lần đầu Docker sẽ tải ảnh và cài các
gói ROS cần thiết.

## Fedora GNOME Wayland + NVIDIA

Trước khi mở container, trên máy chủ chạy:

```bash
xhost +si:localuser:root
```

Sau đó chạy `glxinfo -B` trong container để kiểm tra renderer. Nếu lệnh chưa có,
cài tạm `mesa-utils`; renderer phải là NVIDIA thay vì `llvmpipe`.

Container truyền cả socket Wayland và X11/XWayland. Gazebo Fortress (`gz sim`)
thường chạy ổn định nhất với XWayland, vì vậy nếu cửa sổ không xuất hiện hãy chạy:

```bash
export QT_QPA_PLATFORM=xcb
gz sim
```

## Chạy Gazebo Fortress và kết nối ROS 2

`ros-humble-ros-gz` cài Gazebo Fortress cùng các package kết nối ROS 2. Có thể
khởi động một world rỗng bằng:

```bash
gz sim empty.sdf
```

Trong launch file, dùng `ros_gz_sim` để chạy simulator và `ros_gz_bridge` để
bridge các topic cần thiết, ví dụ `/cmd_vel`, `/scan`, camera, IMU và `/clock`.
Các package mới nên dùng plugin Gazebo (`gz-sim-*`), không dùng `gazebo_ros`
của Gazebo Classic.

## Luồng làm việc tối thiểu

Đặt các package vào `src/`, sau đó:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`--network=host` được bật để discovery DDS giữa simulator, RViz và các node
trên máy chủ không bị vướng cấu hình Docker network. Đây là lựa chọn phù hợp
cho đồ án nội bộ; không nên dùng nguyên trạng cho môi trường cần cô lập mạng.
