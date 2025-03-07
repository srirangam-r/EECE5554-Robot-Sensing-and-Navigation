import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Service(Node):
    def __init__(self):
        super().__init__('srv')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)

    def callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main():
    rclpy.init()
    rclpy.spin(Service())
    rclpy.shutdown()

if __name__ == '__main__':
    main()