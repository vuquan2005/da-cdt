# robot0_navigation

Package điều khiển nhiệm vụ tự hành, định vị tọa độ và điều khiển gắp/trả pallet cho robot Mecanum (`robot0`).

---

## Nhiệm vụ Tự hành Gắp & Trả Pallet (`autonomous_mission.py`)

Node điều khiển máy trạng thái hữu hạn (FSM) thực hiện quy trình tự hành:
1. **WAIT_ODOM**: Đợi khóa tọa độ xuất phát (Home / Spawn pose).
2. **NAV_STAGING**: Di chuyển đến vị trí chuẩn bị trước kệ hàng mục tiêu.
3. **PRE_LIFT_ALIGN**: Căn chỉnh chiều cao tay nâng theo tầng pallet (Level 1 hoặc Level 2).
4. **FORK_INSERT**: Tiến vào xỏ càng nâng vào pallet.
5. **LIFT_PALLET**: Nâng pallet lên độ cao an toàn để di chuyển.
6. **FORK_RETRACT**: Lùi ra khỏi kệ hàng.
7. **NAV_DELIVERY**: Điều hướng đưa pallet về ô chỉ định (Drop-off Zone) hoặc quay về vị trí ban đầu.
8. **LOWER_PALLET**: Hạ và đặt pallet xuống vị trí đích.

### Khởi chạy nhiệm vụ
```bash
# Ví dụ: Gắp pallet ở kệ dưới trái, tầng 1, ngăn trái và đưa về ô Xanh dương
ros2 launch robot0_navigation mission.launch.py rack:=rack_left_bot shelf:=1 slot:=left dropoff:=blue
```

---

## Tọa độ & Động học (`arena_coordinates.py` / `.yaml`)

File module định nghĩa tọa độ chuẩn của:
* Điểm xuất phát của robot (`robot_spawn`)
* Các kệ hàng (`rack_left_bot`, `rack_left_mid`, `rack_left_top`, `rack_bot_mid_left`)
* Các ô trả hàng (`zone_1_blue`, `zone_2_green`, `zone_3_white`, `zone_4_yellow`, `zone_5_red`)
* Thông số chiều cao tay nâng (`level1_insert`, `level1_carry`, `level2_insert`, `level2_carry`, `transit`)

