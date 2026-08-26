# robot0_navigation

Package điều khiển nhiệm vụ tự hành, định vị tọa độ và điều khiển gắp/trả pallet cho robot Mecanum (`robot0`).

---

## Nhiệm vụ Tự hành Gắp & Trả Pallet (`autonomous_mission.py`)

Node điều khiển máy trạng thái hữu hạn (FSM) thực hiện quy trình tự hành:
1. **INIT**: Khởi tạo chiều cao tay nâng và đợi dữ liệu odometry.
2. **NAV_TO_RACK_APPROACH**: Di chuyển từ Home Base đến điểm tiếp cận trước ngăn chứa pallet mục tiêu.
3. **ADJUST_LIFT_APPROACH**: Căn chỉnh chiều cao càng nâng theo tầng kệ (Tầng 1: Bottom 0.0m, Tầng 2: Top 0.12m).
4. **INSERT_FORKS**: Tiến vào xỏ càng nâng chuẩn xác vào khe chân pallet.
5. **LIFT_PALLET**: Nâng pallet lên độ cao mang vác an toàn (Tầng 1: 0.05m, Tầng 2: 0.16m).
6. **RETRACT_FROM_RACK**: Lùi ra khỏi kệ chứa hàng.
7. **NAV_TO_DROPOFF_APPROACH**: Điều hướng đưa pallet đến trạm trả hàng (Drop-off Station: Xanh dương / Đỏ / Xanh lá).
8. **INSERT_DROPOFF**: Tiến vào trạm trả hàng.
9. **LOWER_PALLET**: Hạ tay nâng xuống 0.0m để đặt pallet cố định lên giá đỡ trạm trả hàng.
10. **RETRACT_FROM_DROPOFF**: Lùi ra khỏi trạm trả hàng.
11. **RETURN_HOME**: Di chuyển quay về vị trí Home ban đầu `(0.0, 0.0)`.

### Khởi chạy nhiệm vụ
```bash
# Gắp Pallet CPU (Tầng 1, Trái) và đưa về Trạm Xanh Dương:
ros2 launch robot0_navigation mission.launch.py pallet:=cpu dropoff:=blue

# Gắp Pallet Chip (Tầng 2, Trái) và đưa về Trạm Xanh Lá:
ros2 launch robot0_navigation mission.launch.py pallet:=chip shelf:=2 slot:=left dropoff:=green

# Gắp Pallet Nhôm (Tầng 1, Phải) và đưa về Trạm Đỏ:
ros2 launch robot0_navigation mission.launch.py pallet:=aluminum shelf:=1 slot:=right dropoff:=red
```

---

## Bản đồ Tọa độ Chuẩn (`arena_coordinates.py` / `arena_coordinates.yaml`)

Định nghĩa hệ tọa độ chuẩn hóa:
* **Home Base (Vị trí Bắt đầu / Sạc)**: `(0.0, 0.0, yaw=0.0)`
* **Kệ lấy hàng (Storage Rack)**: `(1.5, 0.0, yaw=1.5708)`
  * *Tầng 1 (Bottom)*: Pallet CPU (Trái: `y=-0.06`), Pallet Nhôm (Phải: `y=0.06`)
  * *Tầng 2 (Top)*: Pallet Chip (Trái: `y=-0.06`), Pallet QR (Phải: `y=0.06`)
* **Trạm trả hàng (Drop-off Stations)**:
  * *Trạm Blue (Bắc)*: `(0.0, 1.2, yaw=0.0)`
  * *Trạm Red (Nam)*: `(0.0, -1.2, yaw=3.1416)`
  * *Trạm Green (Tây)*: `(-1.2, 0.0, yaw=-1.5708)`
