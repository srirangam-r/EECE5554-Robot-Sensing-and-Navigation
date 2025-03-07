import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Client(Node):
    def __init__(self):
        super().__init__('client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    client = Client()
    future = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    client.get_logger().info('Result of add_two_ints: for %d + %d = %d' %(int(sys.argv[1]), int(sys.argv[2]), response.sum))
    rclpy.shutdown()


if __name__ == '__main__':
    main()