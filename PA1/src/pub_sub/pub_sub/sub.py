import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        super().__init__('sub')
        self.subscription = self.create_subscription(String, 'pub_sub_topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main():
    rclpy.init()
    rclpy.spin(Subscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()