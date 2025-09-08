import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        distance = random.uniform(0.0, 1.0)
        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {distance:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
