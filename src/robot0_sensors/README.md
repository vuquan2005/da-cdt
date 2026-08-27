# `robot0_sensors`: Sensor Simulation & Driver Package for Robot0

Gói ROS 2 chuyên trách quản lý, mô phỏng và cung cấp dữ liệu cảm biến cho robot Mecanum `robot0`.

---

## 📡 Các Cảm Biến Cung Cấp

### 1. Vector Geometric Dual Array Line Sensor (`line_sensor_node`)
- **Mô hình:** Mô phỏng toán học 2 thanh cảm biến quang trở/IR phản xạ (Thanh trước +180mm, Thanh sau -180mm, mỗi thanh 8 mắt).
- **Độ chính xác:** Dưới 1mm, không cần render camera sàn Gazebo, tốc độ xử lý cực nhanh (50Hz - 100Hz).
- **Topics cung cấp:**
  - `/line_sensor/raw` (`std_msgs/Int32MultiArray`): Dữ liệu nhị phân (0/1) các mắt.
  - `/line_sensor/analog` (`std_msgs/Float32MultiArray`): Cường độ phản xạ analog [0.0 - 1.0].
  - `/line_sensor/error` (`std_msgs/Float32`): Độ lệch khoảng cách (mét) so với tâm line.
  - `/line_sensor/heading_error` (`std_msgs/Float32`): Độ lệch góc quay (rad) so với line.
  - `/line_sensor/line_detected` (`std_msgs/Bool`): Trạng thái bắt được line.
  - `/line_sensor/junction` (`std_msgs/String`): Phân loại giao cắt (`CROSS`, `T_LEFT`, `T_RIGHT`, `NONE`).
  - `/line_sensor/markers` (`visualization_msgs/MarkerArray`): Hiển thị trực quan 3D trên RViz2.

---

## 🚀 Cách Chạy

### Chạy riêng Line Sensor Node:
```bash
ros2 launch robot0_sensors line_sensor.launch.py
```
