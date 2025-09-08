import rclpy
from rclpy.node import Node
from my_service_pkg.srv import SetVelocity

class VelocityService(Node):
    def __init__(self):
        super().__init__('velocity_service')
        self.srv = self.create_service(SetVelocity, 'set_velocity', self.set_velocity_callback)

    def set_velocity_callback(self, request, response):
        if request.velocity >= 0.0:
            response.result = f"速度{request.velocity}を受け付けました"
        else:
            response.result = "速度は0以上にしてください"
        self.get_logger().info(f'受信: {request.velocity}, 応答: {response.result}')
        return response

def main():
    rclpy.init()
    node = VelocityService()
    rclpy.spin(node)
    rclpy.shutdown()
