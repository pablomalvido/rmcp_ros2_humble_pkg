import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)  # 1 Hz

    def on_timer(self):
        try:
            trans = self.buffer.lookup_transform(
                'base_link',      # target frame
                'link_4',         # source frame
                rclpy.time.Time())
            
            t = trans.transform.translation
            r = trans.transform.rotation
            self.get_logger().info(
                f"link_4 relative to base_link → "
                f"pos=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}), "
                f"rot=({r.x:.2f}, {r.y:.2f}, {r.z:.2f}, {r.w:.2f})"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")

def main():
    rclpy.init()
    node = FrameListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()