# pcgrad Package

Package này cung cấp môi trường mô phỏng robotic arm trong **NVIDIA Isaac Sim** và tích hợp điều khiển thông qua **ROS 2**.

## 📋 Yêu cầu hệ thống

Để chạy được package này, bạn cần có:
- **Ubuntu 22.04**
- **ROS 2 Humble**
- **NVIDIA Isaac Sim** (phiên bản 4.5.0)
- Một chiếc GPU NVIDIA mạnh mẽ hỗ trợ Ray Tracing (RTX).

## 🛠 Cài đặt

1. Di chuyển vào thư mục workspace của bạn:
   ```bash
   cd ~/Project/ros2_ws
   ```

2. Biên dịch package bằng `colcon`:
   ```bash
   colcon build --packages-select pcgrad
   ```

3. Source môi trường:
   ```bash
   source install/setup.bash
   ```

## 🚀 Cách sử dụng

Package này sử dụng một file launch để khởi động đồng thời cả môi trường mô phỏng và node gửi lệnh điều khiển giả lập.

Khởi chạy bằng lệnh sau:
```bash
ros2 launch pcgrad pcgrad_launch.py
```

### Chi tiết các Node

- **`run_environment`**: 
  - Khởi động ứng dụng Isaac Sim.
  - Tải robot từ file URDF tại: `src/robot_description/ARM/robot.urdf`.
  - Tải bàn (`TABLE.usd`) và các vật thể mô phỏng.
  - Thiết lập các liên kết vật lý (Phyisics) và khớp nối (FixedJoint).
  - Thu thập dữ liệu từ camera và thực hiện điều khiển các khớp robot.

- **`joints_state`**:
  - Node này đóng vai trò là "controller" đơn giản.
  - Nó liên tục gửi mảng giá trị `Float64MultiArray` lên topic `/joints_state`.
  - Dữ liệu bao gồm: `[Joint1, Joint2, Joint3, Joint4, Gripper]`.

## 📂 Cấu trúc mã nguồn

- `launch/pcgrad_launch.py`: Cấu hình khởi chạy các node với độ trễ (delay) để đảm bảo Isaac Sim sẵn sàng.
- `pcgrad/run_environment.py`: Mã nguồn chính điều khiển Isaac Sim và ROS 2 node.
- `pcgrad/joints_state.py`: Node Python gửi dữ liệu điều khiển giả lập.
- `setup.py`: Định nghĩa các entry points để ROS 2 có thể nhận diện lệnh thực thi.

## ⚠️ Lưu ý

- Đảm bảo đường dẫn đến tài sản (assets) trong `run_environment.py` là chính xác.
- Isaac Sim cần một khoảng thời gian để khởi tạo GPU, vì vậy file launch đã được thiết lập delay 3 giây trước khi node môi trường bắt đầu chạy.
