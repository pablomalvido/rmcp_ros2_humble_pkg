import rclpy
from rclpy.node import Node
from custom_msgs.srv import CheckEvenOdd


class NumberServiceServer(Node):

    def __init__(self):
        super().__init__('number_service_server')
        self.srv = self.create_service(CheckEvenOdd, 'check_even_odd', self.check_callback)
        self.get_logger().info("Service server ready. Waiting for requests...")

    def check_callback(self, request, response):
        number = request.number
        if number % 2 == 0:
            response.result = f"{number} is EVEN"
        else:
            response.result = f"{number} is ODD"
        self.get_logger().info(f"Received: {number}, Responding: {response.result}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
