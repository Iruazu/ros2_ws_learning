import sys
import rclpy
from rclpy.node import Node
from my_service_pkg.srv import SetVelocity

class VelocityClient(Node):
    def __init__(self):
        super().__init__('velocity_client')
        self.cli = self.create_client(SetVelocity, 'set_velocity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービス待機中...')
        self.req = SetVelocity.Request()

    def send_request(self, velocity):
        self.req.velocity = velocity
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    node = VelocityClient()
    future = node.send_request(float(sys.argv[1]))
    rclpy.spin_until_future_complete(node, future)
    print(f'サーバー応答: {future.result().result}')
    rclpy.shutdown()
