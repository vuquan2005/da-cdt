# 5. Sổ Tay Dòng Lệnh ROS 2 CLI & Hướng Dẫn Debugging

Tài liệu tra cứu nhanh toàn bộ câu lệnh dòng lệnh **ROS 2 CLI**, các công cụ trực quan hóa đồ họa, và các quy chuẩn quốc tế quan trọng nhất.

---

## 💻 1. Bảng Tra Cứu Câu Lệnh Dòng Lệnh (ROS 2 CLI Cheatsheet)

### 🔹 1. Quản lý Node (`ros2 node`)
```bash
# Liệt kê tất cả các node đang chạy
ros2 node list

# Xem chi tiết thông tin 1 node (Subscribers, Publishers, Services, Actions đang kết nối)
ros2 node info /my_node_name
```

### 🔹 2. Quản lý Topic (`ros2 topic`)
```bash
# Liệt kê tất cả các topic
ros2 topic list

# Liệt kê topic kèm kiểu dữ liệu (Message Type)
ros2 topic list -t

# Xem nội dung tin nhắn thời gian thực đang phát trên topic
ros2 topic echo /cmd_vel

# Đo tần số phát của topic (Hz)
ros2 topic hz /odom

# Đo băng thông mạng tiêu thụ của topic (Bytes/s)
ros2 topic bw /camera/image_raw

# Bắn một tin nhắn thủ công lên topic (1 lần)
ros2 topic pub /lift_joint_cmd std_msgs/msg/Float64 "{data: 0.15}" -1

# Bắn tin nhắn liên tục ở tần số 10Hz
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}}" -r 10
```

### 🔹 3. Quản lý Service (`ros2 service`)
```bash
# Liệt kê tất cả các service đang hoạt động
ros2 service list

# Xem kiểu request/response của service
ros2 service type /reset_odometry

# Gọi một service
ros2 service call /reset_odometry std_srvs/srv/Trigger "{}"
```

### 🔹 4. Quản lý Tham Số Động (`ros2 param`)
```bash
# Liệt kê các tham số của tất cả các node
ros2 param list

# Đọc giá trị của 1 tham số
ros2 param get /robot0_controller max_speed

# Thay đổi giá trị tham số ngay khi node đang chạy
ros2 param set /robot0_vision confidence_threshold 0.75

# Xuất toàn bộ tham số của node ra file YAML
ros2 param dump /robot0_controller > controller_dump.yaml
```

### 🔹 5. Quản lý Action (`ros2 action`)
```bash
# Liệt kê các action server đang hoạt động
ros2 action list

# Gửi mục tiêu (Goal) tới một Action Server
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 2.0}}}}"
```

### 🔹 6. Ghi & Phát Lại Dữ Liệu (`ros2 bag`)
```bash
# Ghi lại toàn bộ topic trong hệ thống
ros2 bag record -a

# Chỉ ghi lại một số topic cảm biến cụ thể
ros2 bag record /odom /cmd_vel /camera/image_raw -o my_experiment_bag

# Xem thông tin file bag đã ghi
ros2 bag info my_experiment_bag

# Phát lại dữ liệu đã ghi
ros2 bag play my_experiment_bag
```

---

## 🔍 2. Bộ Công Cụ Trực Quan Hóa & Debugging Đồ Họa

### 1. `rqt_graph` — Sơ đồ Kết nối Mạng Lưới
Cho phép nhìn thấy toàn bộ bức tranh mạng lưới phân tán của hệ thống: Node nào đang phát dữ liệu sang Node nào qua Topic nào:
```bash
rqt_graph
```

### 2. `rqt_plot` — Vẽ Đồ Thị Tín Hiệu 2D Thời Gian Thực
Cực kỳ hữu ích khi tinh chỉnh PID, quan sát sai số bám vạch, hoặc so sánh vận tốc mong muốn vs vận tốc thực tế:
```bash
rqt_plot /line_sensor/lateral_error/data
```

### 3. `tf2_tools` — Kiểm Tra Cây Tọa Độ TF Tree
* **Xuất sơ đồ cây tọa độ ra file PDF:**
  ```bash
  ros2 run tf2_tools view_frames
  # Lệnh trên sẽ sinh ra file frames.pdf hiển thị toàn bộ liên kết giữa các frames
  ```
* **Đo đạc vị trí và góc xoay tức thời giữa 2 frame:**
  ```bash
  ros2 run tf2_ros tf2_echo odom base_link
  ```

### 4. `rviz2` — Trực Quan Hóa 3D Toàn Diện
Công cụ hiển thị 3D mạnh mẽ nhất của ROS 2 để quan sát: Mô hình robot, đám mây điểm LiDAR (PointCloud2), ảnh camera, đường đi dự tính (Path/Trajectory), và các marker đồ họa 3D.

---

## 📏 3. Các Quy Chuẩn Quốc Tế Cốt Lõi Cần Nhớ

### 🔹 REP-103: Quy Ước Trục Tọa Độ & Đơn Vị Đo Lường (Standard Units & Coordinate Conventions)
* **Hệ đơn vị tiêu chuẩn:**
  * Độ dài / Khoảng cách: **mét (m)**
  * Góc / Vận tốc góc: **radian (rad)**, **rad/s**
  * Khối lượng: **kilogram (kg)**
  * Thời gian: **giây (s)**
* **Quy tắc bàn tay phải (Right-Hand Rule):**
  * **Trục X:** Hướng về phía **Trước (Forward)**
  * **Trục Y:** Hướng về phía **Bên Trái (Left)**
  * **Trục Z:** Hướng lên phía **Trên (Up)**
  * **Góc quay Yaw ($\theta$):** Dương khi quay **ngược chiều kim đồng hồ** quanh trục Z.

### 🔹 REP-105: Quy Ước Cây Tọa Độ Robot Di Động (Coordinate Frames for Mobile Platforms)
* `earth`: Tọa độ địa lý GPS toàn cầu (WGS-84).
* `map`: Tọa độ bản đồ cố định trong phòng/nhà xưởng (World frame). Tọa độ không bị trôi dạt.
* `odom`: Tọa độ tích phân chuyển động bánh xe (Local frame). Có tính liên tục cao nhưng trôi dần theo thời gian do trượt bánh.
* `base_footprint`: Hình chiếu 2D của robot trên mặt phẳng sàn ($Z = 0$).
* `base_link`: Gốc cơ khí gắn cố định tại trọng tâm khung gầm robot.
