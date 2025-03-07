import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('pub')
        self.publisher_ = self.create_publisher(String, 'pub_sub_topic', 10)
        self.create_timer(1.0, self.callback)
        self.i = 1

    def callback(self):
        msg = String()
        msg.data = 'Publishing message no.%d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main():
    rclpy.init()
    rclpy.spin(Publisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()