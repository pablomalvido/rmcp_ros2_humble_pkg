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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'

    
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_robotiq'),
        'urdf',
        'multi_arm.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'ros2_control': 'true',
            'ee_id': 'robotiq',
            'gazebo': 'false',
            'effort_gazebo': 'false',
            'arm_ids': "['fr3','fr3']",
            'robot_ips': "['','']",
            'hand': 'false',
            'use_fake_hardware': 'true',
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

    franka_controllers_multi = PathJoinSubstitution(
        [FindPackageShare('franka_robotiq'), 'config', 'multi_controllers.yaml'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[franka_controllers_multi,
                    {'robot_description': robot_description},
                    {'arm_id': 'fr3'},
                    {'load_gripper': 'false'},
                    ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'joints': ['joint_states','franka_gripper/joint_states'],
            'rate': 30,
            'use_robot_description': False,
        }],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
        
    # Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['franka_robot_state_broadcaster'],
    #     parameters=[{'arm_id': arm_id}],
    #     output='screen',
    #     condition=UnlessCondition(use_fake_hardware),
    # ),

    rviz = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', PathJoinSubstitution([
            FindPackageShare('franka_description'), 'rviz', 'visualize_franka.rviz'
        ])],
        output='screen',
    )

    ### ROBOT CONTROLLER ###
    robot_controller = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['joint_impedance_example_controller'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        #joint_state_publisher,
        #rviz,
        controller_manager,
        #joint_state_broadcaster
    ])
