import rclpy
from rclpy.node import Node
from person_msgs.msg import Person  # Personメッセージ型をインポート

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('talker')
        # Publisherの型をPersonに変更
        self.publisher_ = self.create_publisher(Person, 'person', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Person()  # Personメッセージのインスタンスを作成
        msg.name = 'おみくじ太郎'  # 名前を設定
        msg.distance = self.i % 256  # 0-255の範囲で距離を設定
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: name={msg.name} distance={msg.distance}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()