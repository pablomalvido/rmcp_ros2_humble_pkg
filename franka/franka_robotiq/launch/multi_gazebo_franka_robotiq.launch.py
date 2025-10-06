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


def get_robot_description(context: LaunchContext):

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_robotiq'),
        'urdf',
        'multi_arm.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'ros2_control': 'true',
            'hand': 'false', 
            'ee_id': 'robotiq',
            'gazebo': 'true',
            'effort_gazebo': 'false',
            'arm_ids': "['fr3','fr3']",
            'robot_ips': "['','']",
            'hand': 'false',
            'use_fake_hardware': 'false',
            'arm_prefixes': "['left','right']",
            'xyz_values': "['0 0.4 0', '0 -0.4 0']",
            'rpy_values': "['0 0 0', '0 0 0']",
            'number_of_robots': '2',
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
    # Configure ROS nodes for launch

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description)

    #pkg_my_sim = get_package_share_directory('franka_robotiq')
    #world_path = os.path.join(pkg_my_sim, 'worlds', 'gravity_zero.sdf')

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
        #namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             #namespace=namespace,
             arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    left_cartesian_motion_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'left_cartesian_motion_controller'],
        output='screen'
    )

    left_robotiq_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'left_robotiq_gripper_controller'],
        output='screen'
    )

    right_cartesian_motion_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'right_cartesian_motion_controller'],
        output='screen'
    )

    right_robotiq_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'right_robotiq_gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_empty_world,
        robot_state_publisher,
        rviz,
        spawn,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[left_cartesian_motion_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=left_cartesian_motion_controller,
                on_exit=[right_cartesian_motion_controller],
            )
        ),
        
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=right_cartesian_motion_controller,
        #         on_exit=[left_robotiq_gripper_controller],
        #     )
        # ),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=left_robotiq_gripper_controller,
        #         on_exit=[right_robotiq_gripper_controller],
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
