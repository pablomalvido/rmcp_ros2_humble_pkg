#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def robot_description_dependent_nodes_spawner(
        context: LaunchContext,
        robot_ips,
        arm_ids,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper,
        arm_prefixes,
        xyz_values,
        rpy_values,
        number_of_robots):

    robot_ips_str = context.perform_substitution(robot_ips)
    arm_ids_str = context.perform_substitution(arm_ids)
    arm_prefixes_str = context.perform_substitution(arm_prefixes)
    xyz_values_str = context.perform_substitution(xyz_values)
    rpy_values_str = context.perform_substitution(rpy_values)
    number_of_robots_str = context.perform_substitution(number_of_robots)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    franka_xacro_filepath = os.path.join(get_package_share_directory(
        'franka_robotiq'), 'urdf', 'multi_arm.urdf.xacro')
    robot_description = xacro.process_file(franka_xacro_filepath,
                                           mappings={
                                               'ros2_control': 'false',
                                               'arm_ids': arm_ids_str,
                                               'robot_ips': robot_ips_str,
                                               'hand': load_gripper_str,
                                               'use_fake_hardware': use_fake_hardware_str,
                                               'fake_sensor_commands': fake_sensor_commands_str,
                                               'arm_prefixes': arm_prefixes_str,
                                               'xyz_values': xyz_values_str,
                                               'rpy_values': rpy_values_str,
                                               'number_of_robots': number_of_robots_str,
                                           }).toprettyxml(indent='  ')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )
        ]


def generate_launch_description():
    arm_ids_parameter_name = 'arm_ids'
    arm_prefixes_parameter_name = 'arm_prefixes'
    xyz_values_parameter_name = 'xyz_values'
    rpy_values_parameter_name = 'rpy_values'
    number_of_robots_parameter_name = 'number_of_robots'
    robot_ips_parameter_name = 'robot_ips'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    arm_ids = LaunchConfiguration(arm_ids_parameter_name)
    arm_prefixes = LaunchConfiguration(arm_prefixes_parameter_name)
    xyz_values = LaunchConfiguration(xyz_values_parameter_name)
    rpy_values = LaunchConfiguration(rpy_values_parameter_name)
    number_of_robots = LaunchConfiguration(number_of_robots_parameter_name)
    robot_ips = LaunchConfiguration(robot_ips_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    rviz_file = os.path.join(get_package_share_directory('franka_robotiq'), 'rviz',
                             'visualize_franka.rviz')

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ips,
            arm_ids,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            arm_prefixes,
            xyz_values,
            rpy_values,
            number_of_robots])

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            robot_ips_parameter_name,
            default_value="['','','']",
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            arm_ids_parameter_name,
            default_value="['fr3','fr3','fr3']",
            description='ID of the type of arm used. Supported values: fer, fr3, fp3'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='true',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='true',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description='Fake sensor commands. Only valid when "{}" is true'.format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            arm_prefixes_parameter_name,
            default_value="['left','middle','right']",
            description='The prefix of the arms.'),
        DeclareLaunchArgument(
            xyz_values_parameter_name,
            default_value="['0 0.4 0','0 -0.4 0', '0 -1.2 0']",
            description='The prefix of the arms.'),
        DeclareLaunchArgument(
            rpy_values_parameter_name,
            default_value="['0 0 0','0 0 0','0 0 0']",
            description='Orientation of the robots'),
        DeclareLaunchArgument(
            number_of_robots_parameter_name,
            default_value="3",
            description='Number of robots'),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     parameters=[
        #         {'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
        #          'rate': 30}],
        # ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),

        robot_description_dependent_nodes_spawner_opaque_function,
        
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ])

    return launch_description
