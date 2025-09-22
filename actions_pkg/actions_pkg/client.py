import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_msgs.action import Fibonacci
from rclpy.executors import ExternalShutdownException
import time


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._client.wait_for_server()
        self.get_logger().info('Sending goal request...')
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._goal_handle = goal_handle
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Cancelling example: Start a 30 second timer, if the action hasn't finished by then, it preempts it.
        self._timer_cancel = self.create_timer(30.0, self.timer_callback)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: next number = {feedback_msg.feedback.partial_number}')

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal if it exceeds certain time limit
        future = self._goal_handle.cancel_goal_async() #Currently not working for ROS2 Humble
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()        


def main(args=None):
    try:
        rclpy.init(args=args)
        node = FibonacciActionClient()
        node.send_goal(10)  # Example order = 10

        # Main loop can keep doing other things to show the asynchronous behavior of actions
        while rclpy.ok():
            node.get_logger().info("Doing something else...")
            rclpy.spin_once(node, timeout_sec=1)

    except:
        pass

    #rclpy.spin(node)