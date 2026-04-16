import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ArmControllerPublisher(Node):
    def __init__(self):
        super().__init__('arm_controller_publisher')
        
        # 1. Khai báo Publisher dùng kiểu Float64MultiArray
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joints_state', 10)
        
        # 2. Đặt chu kỳ Timer. Với điều khiển tay máy, thường chạy từ 10Hz (0.1s) đến 100Hz (0.01s)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Đang bắt đầu gửi lệnh điều khiển tay máy (3 joints, 1 gripper)...')

    def timer_callback(self):
        # Tạo object chứa message
        msg = Float64MultiArray()
        
        # ------------------------------------------------------------------
        # Tại đây, bạn thay thế bằng code tính toán Động học ngược (IK) 
        # hoặc dữ liệu đọc từ tay cầm/model AI của bạn.
        # Ở đây mình giả lập các giá trị tĩnh:
        # - Góc tính bằng Radian (rad)
        # - Độ dài tính bằng Mét (m)
        # ------------------------------------------------------------------
        joint1_angle = 1.05   # rad
        joint2_angle = 0.50   # rad
        joint3_angle = -0.75  # rad
        joint4_angle = 0.75  # rad
        gripper_length = 0.05 # m (5 cm)
        
        # Đóng gói 4 phần dữ liệu vào thuộc tính .data dưới dạng một list Python
        msg.data = [joint1_angle, joint2_angle, joint3_angle, joint4_angle, gripper_length]
        
        # Gửi lên topic
        self.publisher_.publish(msg)
        
        # In log để dễ theo dõi
        # self.get_logger().info(f'Pub: [J1: {joint1_angle:.2f}, J2: {joint2_angle:.2f}, J3: {joint3_angle:.2f}, Grip: {gripper_length:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()