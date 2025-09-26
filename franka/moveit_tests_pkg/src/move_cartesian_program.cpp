#include <memory>

// MoveIt
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

// tf
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

//cartesian path
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
//#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>
  (
    "move_cartesian_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("move_cartesian_program");
  RCLCPP_INFO(logger, "Starting MoveIt motion from cpp API");

  moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "fr3_arm");

  MoveGroupInterface.setMaxVelocityScalingFactor(0.05);  // 20% of max joint speed. Doesn't work for cartesian plans
  MoveGroupInterface.setMaxAccelerationScalingFactor(0.05);  // 10% of max joint accel. Doesn't work for cartesian plans

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer); 
  rclcpp::sleep_for(std::chrono::seconds(1)); // Wait for TF to populate (optional delay)

  rclcpp::Rate rate(0.5);  // Sleep 2s per cycle (adjust to control cycle speed)
  RCLCPP_INFO(node->get_logger(), "Starting Cartesian motion loop...");

  // Initial end effector position
  geometry_msgs::msg::TransformStamped transformStamped = tf_buffer->lookupTransform("fr3_link0", "fr3_link8", tf2::TimePointZero);
  tf2::Quaternion tf2_quat(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation = msg_quat;
  start_pose.position.x = transformStamped.transform.translation.x;
  start_pose.position.y = transformStamped.transform.translation.y;
  start_pose.position.z = transformStamped.transform.translation.z;

  float offset = 0.15; 
  int index = 1;

  while (rclcpp::ok())
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Move vertically a certain offset
    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.z += (offset*index);
    index *= -1;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_msg;

    const double eef_step = 0.01;      // 1cm resolution
    const double jump_threshold = 0.0; // No jump detection

    double fraction = MoveGroupInterface.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory_msg);

    RCLCPP_INFO(node->get_logger(), "Cartesian path computed %.2f%% of the way", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory_msg;

    MoveGroupInterface.execute(cartesian_plan);

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}