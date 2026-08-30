# robot0_description

Package ROS 2 chứa toàn bộ mô hình mô tả hình học, động học và vật lý của **Robot0** dưới định dạng **URDF / Xacro**, tài nguyên mô hình 3D CAD (.stl), và cấu hình trực quan hóa trên **RViz2**.

---

## 🏗️ Kiến trúc Cấu trúc Xacro Module

Mô hình robot được phân tách thành các module Xacro độc lập nhằm tối ưu hóa tính tái sử dụng và dễ dàng bảo trì:

```
src/robot0_description/urdf/
├── robot0.urdf.xacro          # File tổng hợp chính (Master Entry Point)
├── common_properties.xacro   # Hằng số hình học, vật liệu, thông số quán tính
├── chassis.xacro             # base_footprint, base_link, khung gầm & cụm trụ nâng cố định
├── wheel.xacro               # Macro Xacro sinh 4 bánh Mecanum & trục quay
├── lift.xacro                # Khớp trượt Prismatic & cụm càng nâng hàng (Fork Arm)
├── camera.xacro              # Giá gắn camera nghiêng & Plugin Gazebo Camera Sensor
└── robot0.gazebo.xacro       # Gazebo tags, hệ số ma sát & Joint State Publisher
```

### Chi tiết các file thành phần:

1. **`common_properties.xacro`**:
   * Định nghĩa bán kính bánh xe $r = 0.0487\text{ m}$, nửa khoảng cách cơ sở $l_x = 0.100\text{ m}$, nửa khoảng cách vệt bánh $l_y = 0.1539\text{ m}$.
   * Màu sắc hiển thị RViz: `orange`, `silver`, `dark_grey`, `wheel_grey`.
   * Tọa độ đặt trục nâng ($x=0.128\text{ m}, z=0.0435\text{ m}$) và camera ($x=0.1695\text{ m}, z=0.272\text{ m}$, pitch nghiêng $0.70\text{ rad}$).

2. **`chassis.xacro`**:
   * Định nghĩa `base_footprint` (nằm trên mặt phẳng sàn $Z=0$) liên kết qua fixed joint với `base_link` ($Z=0.05\text{ m}$).
   * Ghép nối các chi tiết CAD của khung gầm: `base_link.stl`, `motor_bracket_*.stl`, các thanh ray trượt và trụ nhôm dẫn hướng (`lift_pipe_1.stl`, `lift_pipe_2.stl`, `lift_top_cap.stl`, `lift_belt.stl`).

3. **`wheel.xacro`**:
   * Macro `mecanum_wheel` nhận tên bánh, vị trí $(x, y, z)$, hướng con lăn và file mesh CAD `motor_jgb37_520_*.stl`.
   * Khởi tạo 4 khớp quay liên tục (`continuous`): `wheel_fl_joint`, `wheel_fr_joint`, `wheel_rl_joint`, `wheel_rr_joint`.

4. **`lift.xacro`**:
   * Khớp trượt tịnh tiến `lift_arm_joint` kiểu `prismatic`, cho phép di chuyển dọc trục $Z$ từ $0.00\text{ m} \to 0.20\text{ m}$ với lực tối đa $100\text{ N}$.
   * Link `lift_arm_link` tích hợp 2 thanh lưỡi càng nâng (fork blades) với hệ số ma sát cao ($\mu = 1.5$) để chống trượt pallet khi nâng hạ và di chuyển tốc độ cao.

5. **`camera.xacro`**:
   * Định nghĩa `camera_link` và `camera_optical_link` theo đúng quy chuẩn ROS REP-103 (X-trước, Y-trái, Z-lên và X-phải, Y-xuống, Z-trước).
   * Tích hợp Gazebo Camera Sensor với độ phân giải $640 \times 480$, góc nhìn FOV $1.047\text{ rad} \approx 60^\circ$, xuất ra topic `/camera/image_raw`.

---

## 🌳 Cây Biến đổi Tọa độ (TF Tree Hierarchy)

Toàn bộ hệ thống liên kết khung tọa độ (TF Frames) được tổ chức chuẩn xác:

```text
odom (Thế giới đo đạc cục bộ)
  │
  └── base_footprint (Hình chiếu robot trên mặt sàn)
        │
        └── base_link (Trọng tâm thân xe, z = +0.050m)
              ├── wheel_fl_link (Trước - Trái, x=+0.100, y=+0.1539)
              ├── wheel_fr_link (Trước - Phải, x=+0.100, y=-0.1539)
              ├── wheel_rl_link (Sau - Trái,   x=-0.100, y=+0.1539)
              ├── wheel_rr_link (Sau - Phải,  x=-0.100, y=-0.1539)
              │
              ├── camera_link (Giá đỡ camera nghiêng pitch 40.1°)
              │     └── camera_optical_link (Chuẩn camera ROS REP-103)
              │
              ├── line_sensor_front_link (Mảng dò line phía trước, x=+0.180m)
              ├── line_sensor_rear_link  (Mảng dò line phía sau,   x=-0.180m)
              ├── line_sensor_left_link  (Mảng dò line bên trái,   y=+0.180m)
              ├── line_sensor_right_link (Mảng dò line bên phải,  y=-0.180m)
              │
              └── lift_arm_link (Trục nâng trượt Z: 0.00m ~ 0.20m)
```

---

## 📊 Thông số Kỹ thuật Vật lý

| Đại lượng | Giá trị | Ghi chú |
| :--- | :--- | :--- |
| **Kích thước bao (Dài $\times$ Rộng $\times$ Cao)** | $320 \times 360 \times 290\text{ mm}$ | Tính đến hết đầu càng nâng |
| **Bán kính bánh xe ($r$)** | $0.0487\text{ m}$ ($48.7\text{ mm}$) | Bánh xe Mecanum con lăn $45^\circ$ |
| **Khối lượng khung gầm (`base_link`)** | $\approx 2.5\text{ kg}$ | Bao gồm động cơ, pin và kết cấu nhôm |
| **Khối lượng cụm nâng (`lift_arm_link`)** | $0.162\text{ kg}$ | Kết cấu nhôm phay CNC |
| **Hành trình nâng ($Z_{\text{lift}}$)** | $0.000\text{ m} \to 0.200\text{ m}$ | Đáp ứng 2 tầng kệ hàng ($0.03\text{m}$ và $0.15\text{m}$) |
| **Góc nghiêng Camera** | $\approx 40.1^\circ$ ($0.70\text{ rad}$) | Tối ưu quan sát khay kệ và pallet gần |

---

## 🚀 Hướng dẫn Kiểm tra & Trực quan hóa (RViz2)

Để kiểm tra mô hình 3D, kiểm tra collision meshes và thử nghiệm kéo thanh trượt điều khiển khớp:

```bash
ros2 launch robot0_description display.launch.py
```

### Các tính năng trong `display.launch.py`:
* Tự động biên dịch xacro `robot0.urdf.xacro` sang chuỗi robot_description.
* Khởi chạy `robot_state_publisher` phát tán `/tf` và `/robot_description`.
* Khởi chạy cửa sổ giao diện `joint_state_publisher_gui` cho phép kéo thanh trượt điều khiển góc xoay 4 bánh xe và độ cao càng nâng `lift_arm_joint`.
* Khởi động `rviz2` với cấu hình định sẵn hiển thị RobotModel, TF Tree và Grid.

---

## 📂 Danh mục Mesh CAD (`meshes/`)

* `base_link.stl`: Tấm đáy chassis chính.
* `lift_arm.stl`: Cụm càng nâng chữ L và lưỡi xỏ pallet.
* `lift_base.stl`, `lift_pipe_1.stl`, `lift_pipe_2.stl`, `lift_top_cap.stl`: Khung trụ nâng thẳng đứng.
* `lift_pulley_20t.stl`, `lift_belt.stl`: Cụm pulley và đai truyền động nâng.
* `motor_bracket_*.stl`: 4 gá kẹp động cơ gắn khung xe (FL, FR, RL, RR).
* `motor_jgb37_520_*.stl`: Động cơ DC giảm tốc JGB37 kèm cụm bánh Mecanum.
