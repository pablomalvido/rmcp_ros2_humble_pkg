#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from std_msgs.msg import *


class InteractiveMarkerDemo(Node):
    def __init__(self, prefix, ns):
        super().__init__(prefix+'interactive_marker_demo')
        self.rate = self.create_rate(50)
        self.prefix = prefix
        self.ns = ns
        self.server = InteractiveMarkerServer(self, self.ns+"/"+self.prefix+'simple_marker')
        self.create_interactive_marker()
    
    def transform_to_pose(self, transform):
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z

        pose.orientation = transform.transform.rotation  # Already a Quaternion
        return pose
    
    def get_init_pose(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Call on_timer function on a set interval
        timer_period = 0.1
        self.init_pose_success = False
        self.timer = self.create_timer(timer_period, self.on_timer)

    def on_timer(self):
        #while not init_pose_success:
        if not self.init_pose_success:
            try:
                #now = rclpy.time.Time()
                self.trans = self.tf_buffer.lookup_transform(self.prefix+'fr3_link0', self.prefix+'fr3_link8', time=rclpy.time.Time())
                self.get_logger().info(f"Link pose: translation={self.trans.transform.translation}")
                self.init_pose_success = True
            except Exception as e:
                self.get_logger().warn(f"Could not get transform: {e}")
                self.trans = None
                self.init_pose_success = False

    def add_6dof_control(self, axis, mode, name):
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        if axis == 'x':
            control.orientation.x = 1.0
        elif axis == 'y':
            control.orientation.y = 1.0
        elif axis == 'z':
            control.orientation.z = 1.0
        control.name = name
        control.interaction_mode = mode
        self.int_marker.controls.append(control)

    def create_interactive_marker(self):
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = self.prefix+"fr3_link0"
        self.int_marker.name = self.prefix+"my_marker"
        self.int_marker.description = "Simple 6-DOF Control"
        self.int_marker.scale = 0.3
        #self.int_marker.header.frame_id = "map"

        #Wait for robot EEF state

        # Position the marker
        # self.int_marker.pose.position.x = 1.0
        # self.int_marker.pose.position.y = 0.0
        # self.int_marker.pose.position.z = 0.0
        self.trans=None
        self.get_init_pose()
        while self.trans is None:
            rclpy.spin_once(self)
        
        init_pose = self.transform_to_pose(self.trans)
        self.int_marker.pose = init_pose
        # Create a helper function to add controls for each axis

        # Add rotation controls
        self.add_6dof_control('x', InteractiveMarkerControl.ROTATE_AXIS, 'rotate_x')
        self.add_6dof_control('y', InteractiveMarkerControl.ROTATE_AXIS, 'rotate_y')
        self.add_6dof_control('z', InteractiveMarkerControl.ROTATE_AXIS, 'rotate_z')

        # Add movement controls
        self.add_6dof_control('x', InteractiveMarkerControl.MOVE_AXIS, 'move_x')
        self.add_6dof_control('y', InteractiveMarkerControl.MOVE_AXIS, 'move_y')
        self.add_6dof_control('z', InteractiveMarkerControl.MOVE_AXIS, 'move_z')

        self.server.insert(self.int_marker)
        self.server.applyChanges()
        #print(self.int_marker)
        self.target_publisher = self.create_publisher(PoseStamped, self.ns+'/'+self.prefix+'cartesian_motion_controller/target_frame', 10)
        self.marker_subscription = self.create_subscription(InteractiveMarkerFeedback, self.ns+"/"+self.prefix+"simple_marker/feedback", self.marker_callback,10)
        #self.marker_subscription
    
    def marker_callback(self,msg):
        marker_pose = msg.pose
        msg = PoseStamped()
        msg.pose = marker_pose
        header = Header()
        header.frame_id = self.prefix+"fr3_link0"
        msg.header = header
        self.target_publisher.publish(msg)


    def process_feedback(self, feedback):
        self.get_logger().info(f"Marker moved to x={feedback.pose.position.x:.2f}, y={feedback.pose.position.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerDemo(prefix='', ns='')
    rclpy.spin(node)
    node.destroy_node()
    #node2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()