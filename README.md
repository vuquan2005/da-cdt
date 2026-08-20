xhost +local:root

### 🚀 Cách kiểm tra & chạy trong Dev Container:

Mở terminal trong devcontainer và chạy các lệnh sau:

    cd /workspaces/ros-cdt

    # 1. Build package
    colcon build --symlink-install

    # 2. Source môi trường
    source install/setup.bash

    # 3. Mở robot hiển thị trên RViz2 với GUI điều khiển khớp
    ros2 launch robot0_description display.launch.py

### 🚀 Hướng dẫn chạy mô phỏng & điều khiển robot:

#### Bước 1: Build lại workspace

    cd /workspaces/ros-cdt
    colcon build --symlink-install
    source install/setup.bash


#### Bước 2: Khởi chạy mô phỏng Gazebo + RViz2

    ros2 launch robot0_description gazebo.launch.py

(Nếu muốn tắt RViz2 để nhẹ máy, có thể chạy: ros2 launch robot0_description gazebo.launch.py rviz:=false)

#### Bước 3: Điều khiển robot di chuyển

• Cách 1: Điều khiển bằng bàn phím (Keyboard Teleop) (mở terminal mới):
ros2 run teleop_twist_keyboard teleop_twist_keyboard

• Cách 2: Điều khiển bằng tay cầm Joystick / Gamepad (mở terminal mới):
ros2 run teleop_twist_joy teleop_node

• Cách 3: Gửi lệnh test trực tiếp: # Cho robot chạy thẳng
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10

    # Cho robot di chuyển ngang (đặc trưng bánh Mecanum)
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {z: 0.0}}" -r 10
