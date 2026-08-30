# Robot0 Controller (`robot0_controller`)

Package ROS 2 chịu trách nhiệm tính toán **Động học nghịch (Inverse Kinematics)** và điều khiển đế di chuyển 4 bánh Mecanum (**4WD Omnidirectional Mobile Base**) cho robot `robot0`.

---

## 📐 Mô hình Động học Bánh Mecanum (Mecanum Kinematics)

Robot0 sử dụng cấu hình 4 bánh xe Mecanum xếp góc $45^\circ$ đối xứng, cho phép di chuyển toàn hướng (Omnidirectional) trên mặt phẳng: tiến/lùi ($v_x$), dạt ngang ($v_y$), và xoay tròn tại chỗ ($\omega_z$).

```
        ▲ +X (Tiến)
        │
   ┌────┴────┐
┌──┤ FL   FR ├──┐
│  │   ▲     │  │
│  │   │     │  │
◄──┼───┼─────┼──┼► +Y (Trái)
│  │   │     │  │
│  │   │     │  │
└──┤ RL   RR ├──┘
   └─────────┘
```

### 1. Ma trận Động học nghịch (Inverse Kinematics)

Công thức toán học chuyển đổi từ vận tốc khung gầm mong muốn $\mathbf{V} = [v_x, v_y, \omega_z]^T$ sang vận tốc góc của từng bánh xe $\mathbf{\Omega} = [\omega_{fl}, \omega_{fr}, \omega_{rl}, \omega_{rr}]^T$:

$$\begin{bmatrix} \omega_{fl} \\ \omega_{fr} \\ \omega_{rl} \\ \omega_{rr} \end{bmatrix} = \frac{1}{r} \begin{bmatrix} 1 & -1 & -(l_x + l_y) \\ 1 & 1 & (l_x + l_y) \\ 1 & 1 & -(l_x + l_y) \\ 1 & -1 & (l_x + l_y) \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

Trong đó:
* $\omega_{fl}$: Vận tốc góc bánh trước-trái (Front-Left) $(\text{rad/s})$
* $\omega_{fr}$: Vận tốc góc bánh trước-phải (Front-Right) $(\text{rad/s})$
* $\omega_{rl}$: Vận tốc góc bánh sau-trái (Rear-Left) $(\text{rad/s})$
* $\omega_{rr}$: Vận tốc góc bánh sau-phải (Rear-Right) $(\text{rad/s})$
* $r$: Bán kính hiệu dụng của bánh xe Mecanum $(0.0487\text{ m})$
* $l_x$: Bán khoảng cách cơ sở trước-sau (Half Wheelbase) $(0.1000\text{ m})$
* $l_y$: Bán khoảng cách vết bánh xe trái-phải (Half Track Width) $(0.1539\text{ m})$
* $k_{\text{geom}} = l_x + l_y = 0.2539\text{ m}$: Hằng số hình học khung gầm

---

## ⚙️ Tính năng Kỹ thuật

1. **Chuyển đổi thời gian thực (Real-time Kinematics Loop):** Chạy ở tần số $50\text{ Hz}$ ($20\text{ ms}$ chu kỳ), đảm bảo độ trễ thấp và phản hồi mượt mà.
2. **Cơ chế Watchdog Tự động ngắt (Auto-timeout Safety):** Tự động phát hiện mất tín hiệu điều khiển `/cmd_vel` quá thời gian cấu hình (mặc định $0.5\text{ s}$) và chủ động gửi lệnh dừng $0.0\text{ rad/s}$ tới toàn bộ 4 bánh để tránh va chạm.
3. **Deadband Filter:** Tự động lọc nhiễu các lệnh vận tốc siêu nhỏ ($|v| < 10^{-4}$) để tránh hiện tượng rung lắc motor khi đứng yên.

---

## 📡 Giao tiếp Topic & ROS 2 Interface

### Subscribed Topics

| Topic | Kiểu Message | Mô tả |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Lệnh vận tốc khung gầm ($v_x, v_y$ tịnh tiến, $\omega_z$ xoay) từ Teleop, Nav2, hoặc Behavior Tree |

### Published Topics

| Topic | Kiểu Message | Đơn vị | Mô tả |
| :--- | :--- | :--- | :--- |
| `/wheel_fl_cmd_vel` | `std_msgs/msg/Float64` | $\text{rad/s}$ | Lệnh vận tốc góc bánh Trước-Trái |
| `/wheel_fr_cmd_vel` | `std_msgs/msg/Float64` | $\text{rad/s}$ | Lệnh vận tốc góc bánh Trước-Phải |
| `/wheel_rl_cmd_vel` | `std_msgs/msg/Float64` | $\text{rad/s}$ | Lệnh vận tốc góc bánh Sau-Trái |
| `/wheel_rr_cmd_vel` | `std_msgs/msg/Float64` | $\text{rad/s}$ | Lệnh vận tốc góc bánh Sau-Phải |

---

## 🛠️ Tham số Cấu hình (`config/controller.yaml`)

| Tham số | Kiểu | Mặc định | Ý nghĩa |
| :--- | :--- | :--- | :--- |
| `wheel_radius` | `double` | `0.0487` | Bán kính bánh xe Mecanum (mét) |
| `wheelbase_lx` | `double` | `0.1000` | Nửa khoảng cách giữa trục trước và trục sau ($L_x / 2$) |
| `track_ly` | `double` | `0.1539` | Nửa khoảng cách giữa 2 vệt bánh trái và phải ($L_y / 2$) |
| `cmd_timeout_sec` | `double` | `0.5` | Thời gian chờ tối đa trước khi tự động ngắt motor (giây) |
| `rate_hz` | `double` | `50.0` | Tần số vòng lặp tính toán động học (Hz) |

---

## 🚀 Hướng dẫn Sử dụng

### 1. Khởi chạy độc lập bằng Launch file
```bash
ros2 launch robot0_controller controller.launch.py
```

### 2. Chạy trực tiếp Node Python
```bash
ros2 run robot0_controller kinematics_node
```

### 3. Kiểm tra bằng cách phát lệnh `/cmd_vel` thủ công

* **Tiến thẳng $0.3\text{ m/s}$:**
  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 20
  ```
* **Dạt ngang sang trái $0.3\text{ m/s}$:**
  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {z: 0.0}}" -r 20
  ```
* **Xoay tại chỗ ngược chiều kim đồng hồ $1.0\text{ rad/s}$:**
  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 1.0}}" -r 20
  ```

* **Quan sát vận tốc các bánh xe xuất ra:**
  ```bash
  ros2 topic echo /wheel_fl_cmd_vel
  ```

---

## 📂 Cấu trúc Thư mục

```text
robot0_controller/
├── CMakeLists.txt / setup.py
├── setup.cfg
├── package.xml
├── README.md
├── config/
│   └── controller.yaml              # Tham số động học và timeout
├── launch/
│   └── controller.launch.py         # Launch file khởi chạy kinematics_node
├── resource/
│   └── robot0_controller
└── robot0_controller/
    ├── __init__.py
    └── kinematics_node.py           # Node tính toán động học nghịch
```
