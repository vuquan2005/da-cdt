# Robot0 Controller (`robot0_controller`)

Package ROS 2 quản lý Động học (Kinematics) và Base Controller cho robot di động Mecanum 4WD (Robot0).

## Chức năng
- **Động học nghịch (Inverse Kinematics):** Đọc lệnh `/cmd_vel` (`geometry_msgs/msg/Twist`) từ bất kỳ nguồn nào (`robot0_teleop`, `robot0_navigation`, `nav2`, v.v.) và chuyển đổi thành vận tốc góc của 4 bánh Mecanum.
- **Tương thích mô phỏng Gazebo:** Xuất lệnh điều khiển vận tốc cho 4 khớp bánh xe:
  - `/wheel_fl_cmd_vel` (`std_msgs/msg/Float64`)
  - `/wheel_fr_cmd_vel` (`std_msgs/msg/Float64`)
  - `/wheel_rl_cmd_vel` (`std_msgs/msg/Float64`)
  - `/wheel_rr_cmd_vel` (`std_msgs/msg/Float64`)
- **Tự động ngắt an toàn (Auto-timeout):** Tự động phát lệnh dừng 0.0 rad/s khi mất tín hiệu `/cmd_vel` quá thời gian định trước (mặc định 0.5s).

## Thông số hình học
- Bán kính bánh xe: $r = 0.0487\text{ m}$
- Bán trục cơ sở (nửa khoảng cách trước - sau): $l_x = 0.1000\text{ m}$
- Bán khoảng cách bánh xe (nửa khoảng cách trái - phải): $l_y = 0.1539\text{ m}$

## Chạy độc lập
```bash
ros2 launch robot0_controller controller.launch.py
```
