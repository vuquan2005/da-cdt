# robot0_teleop

Package xử lý điều khiển thủ công nhận tín hiệu từ joystick gamepad (`sensor_msgs/msg/Joy`) hoặc bàn phím, tính toán động học và xuất lệnh vận tốc di chuyển đa hướng `/cmd_vel` cùng lệnh điều khiển tay nâng `/lift_joint_cmd`.

---

## Khởi chạy điều khiển Gamepad

```bash
# Kiểm tra cổng tay cầm
jstest /dev/input/js0

# Khởi chạy node joy + teleop
ros2 launch robot0_teleop joystick.launch.py
```

### Bảng ánh xạ nút bấm Gamepad

| Phím / Cần gạt | Chức năng | Cơ chế |
| :--- | :--- | :--- |
| `LB / L1` | Deadman Switch | Bắt buộc giữ nút để cho phép robot di chuyển |
| `LT / L2` | Linear Gain | Cò analog điều chỉnh độ lợi tịnh tiến ($0.5\times \to 3.0\times$) |
| `RT / R2` | Angular Gain | Cò analog điều chỉnh độ lợi góc xoay ($1.0\times \to 3.0\times$) |
| `RB / R1` | Turbo Boost | Giữ nút để tăng tức thì vận tốc lên mức tối đa ($3.0\times$) |
| Cần trái (Left Stick) | Tiến / Lùi / Đi ngang | Điều khiển vận tốc tịnh tiến đa hướng $(v_x, v_y)$ |
| Cần phải (Right Stick) | Xoay góc (Yaw) | Điều khiển vận tốc góc $\omega_z$ |
| Nút `Y` / `A` | Nâng / Hạ tay gắp | Điều khiển vị trí trục nâng ($0.0\text{ m} \leftrightarrow 0.18\text{ m}$) |

---

## Điều khiển Bàn phím & Lệnh Topic

```bash
# Điều khiển bằng bàn phím
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Gửi lệnh trực tiếp qua topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {z: 0.0}}" -r 10
ros2 topic pub /lift_joint_cmd std_msgs/msg/Float64 "{data: 0.12}" -1
```

---

## Cấu trúc thư mục

```text
robot0_teleop/
├── CMakeLists.txt / setup.py
├── package.xml
├── config/
│   └── joy_teleop.yaml        # Cấu hình deadzone, mapping nút, vận tốc max
├── launch/
│   └── joystick.launch.py     # Launch joy_node + teleop_node
└── robot0_teleop/
    └── teleop_node.py         # Node xử lý tính toán động học và publish lệnh
```
