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
        robot_ip,
        arm_id,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper,
        arm_prefix):

    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    arm_prefix_str = context.perform_substitution(arm_prefix)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    franka_xacro_filepath = os.path.join(get_package_share_directory(
        'franka_robotiq'), 'urdf', 'fr3_robotiq.urdf.xacro')
    robot_description = xacro.process_file(franka_xacro_filepath,
                                           mappings={
                                               'ros2_control': 'true',
                                               'arm_id': arm_id_str,
                                               'robot_ip': robot_ip_str,
                                               'hand': 'false',
                                               'use_fake_hardware': use_fake_hardware_str,
                                               'fake_sensor_commands': fake_sensor_commands_str,
                                               'arm_prefix': arm_prefix_str,
                                               'gazebo': 'false', 
                                               'ee_id': 'robotiq'
                                           }).toprettyxml(indent='  ')

    franka_controllers = PathJoinSubstitution(
        [FindPackageShare('franka_robotiq'), 'config', 'bringup_controllers.yaml'])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[franka_controllers,
                        {'robot_description': robot_description},
                        {'arm_id': arm_id},
                        {'load_gripper': load_gripper},
                        ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        )
        ]


def generate_launch_description():
    arm_id_parameter_name = 'arm_id'
    arm_prefix_parameter_name = 'arm_prefix'
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    arm_prefix = LaunchConfiguration(arm_prefix_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    rviz_file = os.path.join(get_package_share_directory('franka_robotiq'), 'rviz',
                             'visualize_franka.rviz')
    
    #os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description')) + ':' + os.path.dirname(get_package_share_directory('robotiq_description'))
    #kg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    #gazebo_pkg = get_package_share_directory('gazebo_ros')

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            arm_prefix])

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            default_value='',
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='fr3',
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
            arm_prefix_parameter_name,
            default_value='',
            description='The prefix of the arm.'),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
                 'rate': 30}],
        ),

        robot_description_dependent_nodes_spawner_opaque_function,

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['franka_robot_state_broadcaster'],
        #     parameters=[{'arm_id': arm_id}],
        #     output='screen',
        #     condition=UnlessCondition(use_fake_hardware),
        # ),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
        ),

        ### ROBOT CONTROLLER ###
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_impedance_example_controller'],
            output='screen',
        ),

        ### GRIPPER CONTROLLER ###
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_position_controller", "-c", "/controller_manager"],
        ),

        # #Gazebo Sim
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        #     launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
        # ),

        # # Spawn
        # Node(
        #     package='ros_gz_sim',
        #     executable='create',
        #     arguments=['-topic', '/robot_description'],
        #     output='screen',
        # )

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        #         #os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        #     ),
        #     launch_arguments={
        #         "gui": "true",
        #     }.items(),
        # ),

        # # Spawn robot
        # Node(
        #     package="gazebo_ros",
        #     executable="spawn_entity.py",
        #     arguments=["-entity", "fr3_robotiq", "-topic", "robot_description"],
        #     output="screen",
        # )

    ])

    return launch_description
