import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        self.pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        timer_period = 1.0  # 1秒ごとに送信
        self.timer = self.create_timer(timer_period, self.publish_target)

    def publish_target(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = 1.2
        msg.pose.position.y = 0.5
        msg.pose.position.z = 0.8
        # 現在は回転なし（ワールド軸に揃う）
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.get_logger().info(f'Publish target: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
