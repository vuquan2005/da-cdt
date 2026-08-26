# robot0_navigation

Package xử lý mô phỏng dải cảm biến dò line quang học kép (Dual Array Line Sensor), cung cấp cơ sở dữ liệu tọa độ các vị trí trên sa bàn thi đấu (`arena_coordinates`).

---

## Mô phỏng Cảm biến Dò Line Kép (`line_sensor_node.py`)

Node mô phỏng 2 dải cảm biến quang học độc lập (8 mắt trước + 8 mắt sau) gắn trên khung xe. Node lấy mẫu màu trực tiếp từ texture sàn sa bàn theo vị trí thời gian thực (TF) của robot để tính toán sai lệch bám line.

### Nguyên lý tính toán sai lệch
* **Độ lệch tịnh tiến ngang của tâm robot:**
  $$d_{\text{lateral}} = \frac{e_{\text{front}} + e_{\text{rear}}}{2}$$
* **Góc lệch hướng thân xe so với vạch line:**
  $$\theta_{\text{error}} = \arctan\left(\frac{e_{\text{front}} - e_{\text{rear}}}{L_f + L_r}\right)$$

### Khởi chạy
```bash
ros2 launch robot0_navigation line_sensor.launch.py
```

### Danh sách Topic

| Topic | Kiểu dữ liệu | Ý nghĩa |
| :--- | :--- | :--- |
| `/line_sensor/front/raw` | `std_msgs/msg/Int32MultiArray` | Dữ liệu nhị phân 8 mắt dải trước (0: nền, 1: line) |
| `/line_sensor/front/error` | `std_msgs/msg/Float32` | Độ lệch tâm dải trước $e_{\text{front}}$ (mét) |
| `/line_sensor/front/junction` | `std_msgs/msg/String` | Nhận diện giao lộ dải trước (`NONE`, `CROSS`, `T_LEFT`, `T_RIGHT`, `LOST`) |
| `/line_sensor/rear/raw` | `std_msgs/msg/Int32MultiArray` | Dữ liệu nhị phân 8 mắt dải sau (0: nền, 1: line) |
| `/line_sensor/rear/error` | `std_msgs/msg/Float32` | Độ lệch tâm dải sau $e_{\text{rear}}$ (mét) |
| `/line_sensor/rear/junction` | `std_msgs/msg/String` | Nhận diện giao lộ dải sau (`NONE`, `CROSS`, `T_LEFT`, `T_RIGHT`, `LOST`) |
| `/line_sensor/junction` | `std_msgs/msg/String` | Trạng thái giao lộ tổng hợp của robot |
| `/line_sensor/lateral_error` | `std_msgs/msg/Float32` | Sai lệch ngang tâm xe $d_{\text{lateral}}$ (mét) |
| `/line_sensor/heading_error` | `std_msgs/msg/Float32` | Góc lệch hướng thân xe $\theta_{\text{error}}$ (rad) |
| `/line_sensor/line_detected` | `std_msgs/msg/Bool` | Cờ báo hiệu có phát hiện vạch line |
| `/line_sensor/markers` | `visualization_msgs/msg/MarkerArray` | Marker 3D hiển thị mắt đọc, vector lệch & nhãn giao lộ trên RViz2 |
| `/arena/map` | `nav_msgs/msg/OccupancyGrid` | Bản đồ 2D vạch kẻ sa bàn phục vụ hiển thị |

---

## Tọa độ Sa bàn (`arena_coordinates.py` / `.yaml`)

File module định nghĩa tọa độ chuẩn của các kệ hàng (`rack_left_bot`, `rack_left_mid`,...), các tầng pallet (Level 1, Level 2) và các vùng trả hàng (`blue`, `green`, `red`, `yellow`).
