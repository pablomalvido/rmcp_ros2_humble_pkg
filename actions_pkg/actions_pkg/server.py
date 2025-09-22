import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
import time

from custom_msgs.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback,
            cancel_callback=self.cancel_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal request: order={goal_handle.request.order}')

        order = goal_handle.request.order
        sequence = [0, 1]

        feedback_msg = Fibonacci.Feedback()

        # Publish the two first numbers
        for i in range(2, order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            sequence.append(sequence[i-1] + sequence[i-2])
            feedback_msg.partial_number = sequence[-1]
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Publishing feedback: {feedback_msg.partial_number}')
            time.sleep(1.0)  # Wait 1 second between numbers

        result = Fibonacci.Result()
        result.sequence = sequence[:order]
        self.get_logger().info(f'Goal succeeded with result: {result.sequence}')
        goal_handle.succeed()
        return result
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()
