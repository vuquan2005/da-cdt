# robot0_teleop

Package ROS 2 cung cấp giao diện điều khiển thủ công toàn diện cho robot Mecanum `robot0` thông qua tay cầm **Joystick Gamepad** (Xbox / Logitech F710 / DualShock / Generic Linux Gamepad) hoặc **Bàn phím**, hỗ trợ cơ chế ga mượt tương tự, công tắc an toàn Deadman Switch và điều khiển tay nâng liên tục.

---

## 🎮 Cơ chế Điều khiển Gamepad Nâng cao

```
                 [ LB: Deadman Switch ]      [ RB: Turbo Boost (3.0x) ]
                 [ LT: Analog Linear Gain ]  [ RT: Analog Angular Gain ]
                        ┌──────────────────────────────┐
                        │      [Back]      [Start]     │
       [ D-Pad ]        │                              │     [ Y: Nâng Càng ]
  (Tiến/Lùi/Dạt thô)   │   (L Stick)        (R Stick) │ [ X ]          [ B ]
                        │  (Tiến/Lùi/Dạt)   (Xoay Yaw) │     [ A: Hạ Càng ]
                        └──────────────────────────────┘
```

### 1. Công tắc An toàn Deadman Switch (`LB / L1`)
* **Bắt buộc giữ phím `LB`** khi điều khiển robot di chuyển.
* **Tự động phanh chủ động (Active Auto-Brake):** Ngay khi nhả phím `LB`, node lập tức xuất lệnh dừng $0.0\text{ m/s}$ trên `/cmd_vel` để đảm bảo an toàn tuyệt đối.

### 2. Ga Mượt & Tăng Tốc Analog Triggers (`LT` & `RT`)
* **Cò `LT` (Linear Gain):** Điều chỉnh độ lợi vận tốc tịnh tiến liên tục từ $0.5\times$ (chế độ di chuyển tinh chỉnh vi mô gần kệ) đến $3.0\times$ (tốc độ cao).
* **Cò `RT` (Angular Gain):** Điều chỉnh độ lợi vận tốc góc xoay liên tục từ $1.0\times$ đến $3.0\times$.
* **Phím `RB` (Digital Turbo Boost):** Giữ nút để kích hoạt ngay lập tức vận tốc cực đại ($3.0\times$) cho cả tịnh tiến và xoay.

### 3. Điều khiển Càng nâng Tự động đồng bộ (`Y` & `A`)
* **Nâng càng (Phím `Y`):** Tăng độ cao càng nâng với tốc độ $0.10\text{ m/s}$ đến giới hạn trên $0.20\text{ m}$.
* **Hạ càng (Phím `A`):** Giảm độ cao càng nâng với tốc độ $0.10\text{ m/s}$ về vị trí sát sàn $0.00\text{ m}$.
* **Auto-Sync:** Tự động đồng bộ với vị trí phản hồi từ `/joint_states` khi ngừng ấn nút, ngăn ngừa sai lệch giữa lệnh điều khiển và trạng thái thực của cơ cấu trượt.

---

## 🕹️ Bảng Ánh xạ Phím Tay cầm (Gamepad Mapping)

| Phím / Trục | Mã Index | Chức năng | Hành vi |
| :--- | :---: | :--- | :--- |
| **`LB` (L1)** | `Button 4` | **Deadman Switch** | **Bắt buộc giữ để cho phép robot chuyển động** |
| **`LT` (L2)** | `Axis 2` | **Linear Gain** | Cò analog: Nhẹ = $0.5\times$, Kịch sàn = $3.0\times$ |
| **`RT` (R2)** | `Axis 5` | **Angular Gain** | Cò analog: Nhẹ = $1.0\times$, Kịch sàn = $3.0\times$ |
| **`RB` (R1)** | `Button 5` | **Turbo Boost** | Giữ nút để đạt vận tốc tối đa $3.0\times$ |
| **Cần Trái (Trục Y)** | `Axis 1` | **Tiến / Lùi ($v_x$)** | Đẩy lên: Tiến ($+v_x$), Kéo xuống: Lùi ($-v_x$) |
| **Cần Trái (Trục X)** | `Axis 0` | **Dạt Ngang ($v_y$)** | Gạt trái: Dạt trái ($+v_y$), Gạt phải: Dạt phải ($-v_y$) |
| **Cần Phải (Trục X)**| `Axis 3` | **Xoay tròn ($\omega_z$)**| Gạt trái: Xoay ngược CKĐ, Gạt phải: Xoay thuận CKĐ |
| **Phím `Y`** | `Button 3` | **Nâng Càng** | Giữ nút để nâng càng lên cao ($Z_{\text{max}} = 0.20\text{ m}$) |
| **Phím `A`** | `Button 0` | **Hạ Càng** | Giữ nút để hạ càng xuống sàn ($Z_{\text{min}} = 0.00\text{ m}$) |

---

## 📡 ROS 2 Interface

### Subscribed Topics
| Topic | Kiểu Message | Mô tả |
| :--- | :--- | :--- |
| `/joy` | `sensor_msgs/msg/Joy` | Luồng tín hiệu thô từ tay cầm gamepad |
| `/joint_states` | `sensor_msgs/msg/JointState` | Dữ liệu phản hồi góc/vị trí khớp thực tế từ robot |

### Published Topics
| Topic | Kiểu Message | Mô tả |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Lệnh vận tốc đa hướng gửi đến controller |
| `/lift_joint_cmd` | `std_msgs/msg/Float64` | Lệnh vị trí độ cao nâng/hạ càng ($0.0 \to 0.20\text{ m}$) |

---

## 🛠️ Tham số Cấu hình (`config/joy_teleop.yaml`)

| Tham số | Kiểu | Mặc định | Mô tả |
| :--- | :--- | :--- | :--- |
| `deadzone` | `double` | `0.05` | Ngưỡng triệt tiêu nhiễu cần gạt analog ($5\%$) |
| `require_enable_button` | `bool` | `true` | Yêu cầu giữ nút Deadman Switch khi điều khiển |
| `enable_button` | `int` | `4` | Index của nút Deadman Switch (`LB`) |
| `scale_linear_x` | `double` | `0.5` | Vận tốc tịnh tiến tiến/lùi cơ bản ($\text{m/s}$) |
| `scale_linear_y` | `double` | `0.5` | Vận tốc dạt ngang cơ bản ($\text{m/s}$) |
| `scale_angular_z` | `double` | `1.2` | Vận tốc góc xoay cơ bản ($\text{rad/s}$) |
| `turbo_multiplier` | `double` | `3.0` | Hệ số khuếch đại vận tốc khi kích hoạt Turbo |
| `lift_speed` | `double` | `0.10` | Tốc độ nâng hạ càng ($0.10\text{ m/s}$) |
| `lift_min` / `lift_max` | `double` | `0.0` / `0.20` | Giới hạn hành trình nâng hạ (mét) |

---

## 🚀 Hướng dẫn Khởi chạy & Kiểm tra

### 1. Kiểm tra cổng nhận diện Gamepad trong Linux:
```bash
# Kiểm tra file thiết bị
ls -l /dev/input/js*

# Kiểm tra tín hiệu các nút bấm
jstest /dev/input/js0
```

### 2. Khởi chạy cụm Node Teleop:
```bash
ros2 launch robot0_teleop joystick.launch.py
```

### 3. Điều khiển thay thế bằng Bàn phím (Keyboard Teleop):
Khi không có tay cầm vật lý, bạn có thể điều khiển trực tiếp qua bàn phím:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*(Phím `i`: tiến, `,`: lùi, `j`: xoay trái, `l`: xoay phải, `u/o/m/.`: dạt chéo).*
