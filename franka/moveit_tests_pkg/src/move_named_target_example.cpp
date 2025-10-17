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

  MoveGroupInterface.setMaxVelocityScalingFactor(0.5);  
  MoveGroupInterface.setMaxAccelerationScalingFactor(0.5);  

  MoveGroupInterface.setNamedTarget("extended");

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

  MoveGroupInterface.setNamedTarget("ready");

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  auto const outcome2 = static_cast<bool>(MoveGroupInterface.plan(plan2));

  //Execute the plan
  if(outcome2)
  {
    MoveGroupInterface.execute(plan2);
  }
  else
  {
    RCLCPP_ERROR(logger, "Error not able to execute");
  }

  rclcpp::shutdown();
  return 0;
}