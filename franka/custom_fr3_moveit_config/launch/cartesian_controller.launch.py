import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(robot_name="fr3", package_name="custom_fr3_moveit_config")
        .robot_description(
            file_path="config/fr3.urdf.xacro",
            mappings={
                'use_fake_hardware': 'true',
                'ros2_control': 'true'
            },
        )
        .robot_description_semantic(file_path="config/fr3.srdf.xacro")
        .trajectory_execution(file_path="config/fr3_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("custom_fr3_moveit_config"), "rviz"
    )
    rviz_moveit_config = os.path.join(rviz_base, "moveit_empty.rviz")
    rviz_moveit = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_moveit_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    rviz_cartesian_config = os.path.join(rviz_base, "visualize_robot.rviz")
    rviz_cartesian = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_cartesian_config],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "fr3_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("custom_fr3_moveit_config"),
        "config",
        "fr3_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    fr3_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", "-c", "/controller_manager"],
    )

    fr3_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_gripper", "-c", "/controller_manager"],
    )

    cartesian_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'cartesian_motion_controller'],
        output='screen'
    )

    marker_node = Node(
        package='python_scripts_pkg',
        executable='interactive_marker_rviz',
        name='interactive_marker_rviz',
        output='screen'
    )

    return LaunchDescription(
        [
            rviz_moveit,
            #static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            fr3_arm_controller_spawner,
            cartesian_controller,
            fr3_hand_controller_spawner,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher,
                    on_start=[static_tf_node],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher,
                    on_start=[marker_node],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=marker_node,
                    on_start=[rviz_cartesian],
                )
            ),
        ]
    )
