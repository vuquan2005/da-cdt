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
