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
    "move_target_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("move_target_program");
  RCLCPP_INFO(logger, "Starting MoveIt motion from cpp API");

  moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "fr3_arm");

  MoveGroupInterface.setMaxVelocityScalingFactor(0.05);  // 20% of max joint speed
  MoveGroupInterface.setMaxAccelerationScalingFactor(0.05);  // 10% of max joint accel

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  rclcpp::sleep_for(std::chrono::seconds(1)); // Wait for TF to populate (optional delay)

  rclcpp::Rate rate(0.5);  // Sleep 2s per cycle (adjust to control cycle speed)

  RCLCPP_INFO(node->get_logger(), "Starting set target pose motion loop...");

  geometry_msgs::msg::TransformStamped transformStamped = tf_buffer->lookupTransform("fr3_link0", "fr3_link8", tf2::TimePointZero);

  tf2::Quaternion tf2_quat2(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  geometry_msgs::msg::Quaternion msg_quat2 = tf2::toMsg(tf2_quat2);

  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation = msg_quat2;
  start_pose.position.x = transformStamped.transform.translation.x;
  start_pose.position.y = transformStamped.transform.translation.y;
  start_pose.position.z = transformStamped.transform.translation.z;

  float offset = 0.15; 
  int index = 1;

  while (rclcpp::ok())
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Move down by 10 cm
    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.z += (offset*index);
    index *= -1;

    MoveGroupInterface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const outcome = static_cast<bool>(MoveGroupInterface.plan(plan));

    //Execute the plan
    if(outcome)
    {
      MoveGroupInterface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Error not able to execute");
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}