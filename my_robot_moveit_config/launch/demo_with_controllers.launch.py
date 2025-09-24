import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():

    # DEFINE MOVEIT CONFIGURATION OBJECT
    moveit_config = (
        MoveItConfigsBuilder("FASTbot", package_name="my_robot_moveit_config")
        .robot_description(file_path="config/FASTbot.urdf.xacro")
        .robot_description_semantic(file_path="config/FASTbot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # LAUNCH CONTROLLER MANAGER WITH THE REQUIRED ROS2 CONTROLLERS
    ros2_controllers_path = os.path.join(
        get_package_share_directory("my_robot_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # CONTROLLERS
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'arm_trajectory_controller'],
        output='screen'
    )

    # LAUNCH MOVE GROUP NODE (MOVEIT CORE FUNCTIONALITY)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RVIZ VISUALIZATION ENVIRONMENT
    rviz_file = os.path.join(get_package_share_directory(
        'my_robot_moveit_config'), 'config', 'moveit.rviz')
    
    rviz = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': moveit_config.robot_description}],
    )


    return LaunchDescription(
        [
        robot_state_publisher,
        ros2_control_node,
        load_joint_state_broadcaster,
        load_arm_trajectory_controller,
        move_group_node,
        rviz,
        ]
    )