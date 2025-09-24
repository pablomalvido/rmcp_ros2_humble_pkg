import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class FrameBroadcaster(Node):
    def __init__(self):
        super().__init__('frame_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.broadcast_callback)  # 1 Hz
        self.z = -1.0
        self.direction = 0.1

    def broadcast_callback(self):
        # Update z position
        self.z += self.direction
        if self.z >= 1.0 or self.z <= -1.0:
            self.direction *= -1  # bounce

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "link_2"    # parent frame
        t.child_frame_id = "new_link"     # child frame we’re creating
        t.transform.translation.x = 2.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)
        self.get_logger().info(f"Broadcasted TF: z={self.z:.2f}")

def main():
    rclpy.init()
    node = FrameBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
