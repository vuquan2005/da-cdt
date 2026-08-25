# robot0_description

Package chứa mô hình mô tả robot định dạng Xacro/URDF, tài nguyên CAD 3D (.stl) và cấu hình hiển thị trên RViz2 cho robot di động 4 bánh Mecanum tích hợp cơ cấu tay nâng dạng trượt (`robot0`).

---

## Cấu trúc Mô hình Xacro

```text
urdf/
├── robot0.urdf.xacro          # File tổng hợp chính (Master Entry)
├── common_properties.xacro   # Hằng số, vật liệu (Colors/Materials), thông số bánh xe
├── chassis.xacro             # base_footprint, base_link, khung gầm và trụ nâng cố định
├── wheel.xacro               # Macro xacro và khởi tạo 4 bánh Mecanum
├── lift.xacro                # Khớp trượt prismatic và link tay nâng
├── camera.xacro              # Tọa độ và Gazebo Camera Sensor Plugin
└── robot0.gazebo.xacro       # Gazebo plugins (odometry, kinematics, joint controllers)
```

---

## Thông số kỹ thuật

| Thành phần | Đặc tả kỹ thuật |
| :--- | :--- |
| **Khung gầm** | 4 bánh Mecanum toàn hướng (Omnidirectional) |
| **Khớp nâng (`lift_joint`)** | Khớp trượt (Prismatic), hành trình $0.0\text{ m} \to 0.18\text{ m}$ theo trục Z |
| **Cảm biến** | Camera RGB (`camera_link`), 2 dải cảm biến dò line quang học (trước/sau) |
| **Gốc tọa độ** | `base_footprint` (mặt sàn) $\to$ `base_link` ($z = 0.05\text{ m}$) |

---

## Khởi chạy hiển thị mô hình (RViz2)

Khởi chạy `robot_state_publisher` và `joint_state_publisher_gui` để kiểm tra cây biến đổi tọa độ (TF Tree) và góc quay các khớp:

```bash
ros2 launch robot0_description display.launch.py
```

---

## Cây biến đổi tọa độ (TF Tree)

```text
odom
  └── base_footprint
        └── base_link
              ├── wheel_fl_link
              ├── wheel_fr_link
              ├── wheel_rl_link
              ├── wheel_rr_link
              ├── camera_link
              ├── line_sensor_front_link
              ├── line_sensor_rear_link
              └── lift_base_link
                    └── lift_link (Z: 0.0 ~ 0.18m)
```
