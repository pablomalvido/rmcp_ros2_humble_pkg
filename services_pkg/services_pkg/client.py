import sys
import rclpy
from rclpy.node import Node
from custom_msgs.srv import CheckEvenOdd


class NumberServiceClient(Node):

    def __init__(self):
        super().__init__('number_service_client')
        self.cli = self.create_client(CheckEvenOdd, 'check_even_odd')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = CheckEvenOdd.Request()

    def send_request(self, number):
        self.req.number = number
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    node = NumberServiceClient()
    if len(sys.argv) < 2:
        node.get_logger().info("Usage: ros2 run number_checker number_service_client <number>")
        return

    number = int(sys.argv[1])
    future = node.send_request(number)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Result: {future.result().result}")
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()
