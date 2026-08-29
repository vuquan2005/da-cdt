# `robot0_navigation`: Behavior Trees Pallet Retrieval & Navigation

Gói ROS 2 điều khiển tự động hóa quy trình **Lấy Pallet từ Kệ và Vận chuyển đến Vị trí Đích** cho robot Mecanum (`robot0`) dựa trên kiến trúc **Behavior Trees (Cây hành vi)**.

---

## 🌲 Kiến trúc Cây Hành Vi (Behavior Tree Architecture)

Hệ thống cây hành vi được thiết kế theo cấu trúc module độc lập, điều phối luồng tuần tự qua **Blackboard** dùng chung:

```mermaid
graph TD
    Master["Selector: Master Search & Retrieve Mission"] --> Flow["1. Sequence: Search & Retrieve Flow"]
    Master --> Abort["2. Sequence: Abort & Return Home (Nếu không thấy ở cả 2 kệ)"]

    %% Flow branches
    Flow --> Init["1A. Sequence: Initialization (Odom + Transit Lift)"]
    Flow --> SearchSel["1B. Selector: Search Racks Selector"]

    %% Try Rack 1
    SearchSel --> TryR1["Sequence: Try Rack 1"]
    TryR1 --> NavR1["Nav to Rack 1 (X=-1.500m)"]
    TryR1 --> ScanR1{"YOLO Scan Rack 1<br/>(Tìm loại hàng mục tiêu)"}
    ScanR1 -->|Found| PickR1["Pick Pallet (Shift slot + Lift + Insert + Retract)"]
    PickR1 --> DeliverR1["Deliver to Dynamic Drop-off Zone (Theo loại hàng)"]
    DeliverR1 --> PlaceR1["Place Pallet & Backoff"]
    PlaceR1 --> ReturnR1["Return to Home Base"]

    %% Try Rack 2 (Fallback if not found at Rack 1)
    SearchSel --> TryR2["Sequence: Try Rack 2 (Nếu Rack 1 không có)"]
    TryR2 --> NavR2["Nav from Rack 1 to Rack 2 (Qua nhánh rẽ X=-0.400m)"]
    TryR2 --> ScanR2{"YOLO Scan Rack 2<br/>(Tìm loại hàng mục tiêu)"}
    ScanR2 -->|Found| PickR2["Pick Pallet (Shift slot + Lift + Insert + Retract)"]
    PickR2 --> DeliverR2["Deliver to Dynamic Drop-off Zone (Theo loại hàng)"]
    DeliverR2 --> PlaceR2["Place Pallet & Backoff"]
    PlaceR2 --> ReturnR2["Return to Home Base"]

    %% Abort flow
    Abort --> AbortLog["Log: Không tìm thấy pallet ở cả 2 kệ!"]
    Abort --> AbortNav["Nav Home from Rack 2 (Trở về vạch xuất phát an toàn)"]
```

---

## 🗺️ Tọa độ Sa bàn & Kệ Pallet (Single Source of Truth)

| Đối tượng | Vị trí | Tọa độ Thế giới $(X, Y, Z)$ | Tầng / Vị trí |
| :--- | :--- | :--- | :--- |
| **Start Base** | Xuất phát | `X = -0.985, Y = 0.640, Z = 0.080` ($Yaw = \pi$) | Sàn |
| **Rack 1** | Kệ dưới | `X = -1.894, Y = 0.640` ($Yaw = \pi/2$) | - |
| - *Pallet Nhôm* | Rack 1 | `X = -1.894, Y = 0.580, Z = 0.0285` | **Tầng 1 (Dưới - Trái)** |
| - *Pallet CPU* | Rack 1 | `X = -1.894, Y = 0.700, Z = 0.1485` | **Tầng 2 (Trên - Phải)** |
| **Rack 2** | Kệ giữa | `X = -1.894, Y = 0.000` ($Yaw = \pi/2$) | - |
| - *Pallet QR* | Rack 2 | `X = -1.894, Y = -0.060, Z = 0.0285` | **Tầng 1 (Dưới - Trái)** |
| - *Pallet Chip* | Rack 2 | `X = -1.894, Y = 0.060, Z = 0.1485` | **Tầng 2 (Trên - Phải)** |
| **Drop-off 1** | Vùng Nhôm | `X = 0.70, Y = 0.64` (Tiếp cận: `X = 0.55`) | Xanh lam |
| **Drop-off 2** | Vùng CPU | `X = 0.70, Y = 0.22` (Tiếp cận: `X = 0.55`) | Xanh lá |
| **Drop-off 3** | Vùng QR | `X = 0.70, Y = -0.22` (Tiếp cận: `X = 0.55`) | Vàng |
| **Drop-off 4** | Vùng Chip | `X = 0.70, Y = -0.64` (Tiếp cận: `X = 0.55`) | Đỏ |

---

## 🚀 Hướng Dẫn Sử Dụng

### 1. Build Package
```bash
colcon build --symlink-install --packages-select robot0_navigation
source install/setup.bash
```

### 2. Khởi chạy Mô phỏng Sa bàn (Gazebo)
```bash
ros2 launch robot0_gazebo gazebo.launch.py
```

### 3. Chạy Kịch bản Behavior Tree

#### Cách 1: Chọn theo loại Pallet (Tự động xác định Kệ, Tầng, Vị trí trả hàng)
* **Gắp Pallet Nhôm (Tầng 1, Kệ 1 $\to$ Vùng 1):**
  ```bash
  ros2 launch robot0_navigation pallet_mission_bt.launch.py pallet:=aluminum
  ```
* **Gắp Pallet CPU (Tầng 2, Kệ 1 $\to$ Vùng 2):**
  ```bash
  ros2 launch robot0_navigation pallet_mission_bt.launch.py pallet:=cpu
  ```
* **Gắp Pallet QR Code (Tầng 1, Kệ 2 $\to$ Vùng 3):**
  ```bash
  ros2 launch robot0_navigation pallet_mission_bt.launch.py pallet:=qr
  ```
* **Gắp Pallet Chip (Tầng 2, Kệ 2 $\to$ Vùng 4):**
  ```bash
  ros2 launch robot0_navigation pallet_mission_bt.launch.py pallet:=chip
  ```

#### Cách 2: Tùy chỉnh chi tiết Kệ, Tầng, Khay và Vị trí trả
```bash
ros2 launch robot0_navigation pallet_mission_bt.launch.py rack:=rack_1 shelf:=1 slot:=left dropoff:=dropoff_1
```
*(Hoặc `dropoff:=home` nếu muốn mang pallet về vị trí ban đầu).*

---

## 🛠️ Trực quan hóa Behavior Tree thời gian thực

Trong quá trình chạy, node sẽ tự động in trạng thái cây ra Terminal với mã màu:
- **`[SUCCESS]` (Xanh lá)**: Node đã hoàn thành mục tiêu.
- **`[RUNNING]` (Vàng)**: Node đang tích cực điều khiển robot.
- **`[FAILURE]` (Đỏ)**: Node gặp sự cố / timeout.
- **`[INVALID]` (Xám)**: Node chưa được kích hoạt tới.

---

## 🔮 Hướng Phát Triển Tự Động Hóa Nâng Cao

Do cây hành vi được thiết kế theo dạng **Blackboard & Modular Actions**, bạn có thể dễ dàng mở rộng:
1. **Tích hợp YOLO Vision (`robot0_vision`)**: Thêm node `VisionDetectPalletAction` trước bước `Insert_Fork` để bù trừ sai lệch góc hoặc vị trí thực tế của pallet.
2. **Tích hợp Dò Line (`line_sensor_node`)**: Thay thế `NavigateToPoseAction` bằng `FollowLineAction` trên các trục line chính.
3. **Tích hợp Nav2**: Thêm `Nav2GoalAction` khi môi trường có chướng ngại vật động.
