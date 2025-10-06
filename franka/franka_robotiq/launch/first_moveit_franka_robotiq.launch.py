# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def get_robot_description(context: LaunchContext):

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_robotiq'),
        'urdf',
        'fr3_robotiq.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': 'fr3', #arm_id_str,
            'hand': 'true', #load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'robotiq', #franka_hand_str,
            'gazebo_effort': 'false',
            'use_fake_hardware': 'false',
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]



def generate_launch_description():

    # joint_controllers_file = os.path.join(
    #     get_package_share_directory('ur_yt_sim'), 'config', 'ur5_controllers_gripper.yaml'
    # )

    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="robotiq_gripper_moveit_config") #CHANGE PACKAGE NAME
        .robot_description(file_path="config/fr3_robotiq.urdf.xacro", mappings={ #Points to the real URDF
            'arm_id': 'fr3', #arm_id_str,
            'hand': 'true', #load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'robotiq', #franka_hand_str,
            'gazebo_effort': 'false',
            'use_fake_hardware': 'false',
        }) 
        .robot_description_semantic(file_path="config/fr3.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description],
        output='screen'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("robotiq_gripper_moveit_config"), #CHANGE PACKAGE NAME
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path, '-f', 'world'],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    use_sim_time={"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )
    ###########################################

    # Get robot description
    # robot_state_publisher = OpaqueFunction(
    #     function=get_robot_description)

    # pkg_my_sim = get_package_share_directory('franka_robotiq')
    # world_path = os.path.join(pkg_my_sim, 'worlds', 'gravity_zero.sdf')

    # gazebo_world = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #     ),
    #     launch_arguments={'gz_args': f'{world_path}'}.items()
    # )

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        #launch_arguments={'gz_args': f'{world_path}'+' -r', }.items(),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Visualize in RViz
    # rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
    #                          'visualize_franka.rviz')
    # rviz = Node(package='rviz2',
    #          executable='rviz2',
    #          name='rviz2',
    #          namespace='',
    #          arguments=['--display-config', rviz_file, '-f', 'world'],
    # )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    gravity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'gravity_compensation_example_controller'],
        output='screen'
    )

    cartesian_motion_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'cartesian_motion_controller'],
        output='screen'
    )

    joint_position_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'joint_position_example_controller'],
        output='screen'
    )

    moveit_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'robotiq_gripper_controller'],
        output='screen'
    )

    moveit_franka_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_empty_world,
        robot_state_publisher,
        rviz_node,
        spawn,
        move_group_node,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[moveit_franka_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=moveit_franka_controller,
                on_exit=[moveit_gripper_controller],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[cartesian_motion_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=cartesian_motion_controller,
        #         on_exit=[gravity_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[joint_position_example_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_position_example_controller,
        #         on_exit=[robotiq_gripper_controller],
        #     )
        # ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     namespace=namespace,
        #     parameters=[
        #         {'source_list': ['joint_states'],
        #          'rate': 30}],
        # ),
    ])
