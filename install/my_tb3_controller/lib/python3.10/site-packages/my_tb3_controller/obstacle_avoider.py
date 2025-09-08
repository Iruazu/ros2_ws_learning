import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys

class ObstacleAvoiderNode(Node):
    """
    Lidarセンサーで前方の障害物を検知し、
    障害物がなければ前進、あれば回転するシンプルな障害物回避ノード
    """
    def __init__(self):
        """
        ノードの初期化
        """
        super().__init__('obstacle_avoider_node')
        
        # Publisherの作成: /cmd_velトピックにTwist型のメッセージを配信
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriberの作成: /scanトピックからLaserScan型のメッセージを購読
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # 0.1秒ごとにtimer_callback関数を呼び出すタイマーの作成
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 障害物を検知したかどうかを管理するフラグ
        self.is_obstacle_detected = False
        
        self.get_logger().info('Obstacle Avoider Node has been started successfully.')

    def scan_callback(self, msg):
        """
        /scanトピックからデータを受け取るたびに呼び出されるコールバック関数
        """
        # TurtleBot3のLidar(360度)の正面30度の範囲をチェック
        # ranges[0:15]が右前15度、ranges[-15:]が左前15度に対応
        front_ranges = msg.ranges[0:15] + msg.ranges[-15:]
        
        # 取得した範囲内の無効な値（infやnan）を除外
        valid_ranges = [r for r in front_ranges if r > 0.0 and r != float('inf')]
        
        # もし有効なデータがなければ、何もしない
        if not valid_ranges:
            self.get_logger().warn('No valid laser scan data in front.')
            return
            
        # 有効なデータの中から最小値（最も近い障害物までの距離）を取得
        min_distance = min(valid_ranges)
        
        # デバッグ用に最小距離をログに出力
        self.get_logger().info(f'Minimum distance in front: {min_distance:.2f} m')

        # しきい値（ここでは0.4m）より近ければ、障害物検知フラグをTrueにする
        if min_distance < 0.4:
            self.is_obstacle_detected = True
        else:
            self.is_obstacle_detected = False

    def timer_callback(self):
        """
        0.1秒ごとに呼び出され、ロボットへの速度指令を決定し配信する関数
        """
        # 送信するメッセージの器を準備
        twist_msg = Twist()
        
        # 障害物検知フラグの状態に応じて、行動を決定
        if self.is_obstacle_detected:
            # 障害物があれば、その場で回転する
            twist_msg.linear.x = 0.0  # 前進は停止
            twist_msg.angular.z = 0.5  # 0.5 rad/s で左回転
            self.get_logger().info('Obstacle detected! Turning...')
        else:
            # 障害物がなければ、前進する
            twist_msg.linear.x = 0.2  # 0.2 m/s で前進
            twist_msg.angular.z = 0.0  # 回転は停止
            self.get_logger().info('No obstacle. Moving forward...')
        
        # 決定した速度指令を配信
        self.publisher_.publish(twist_msg)

def main(args=None):
    """
    プログラムのエントリーポイント（玄関）
    """
    rclpy.init(args=args)
    try:
        obstacle_avoider_node = ObstacleAvoiderNode()
        rclpy.spin(obstacle_avoider_node)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_avoider_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()