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


def robot_description_dependent_nodes_spawner(
        context: LaunchContext,
        robot_ip_,
        arm_id_,
        use_fake_hardware_,
        fake_sensor_commands_,
        load_gripper_,
        arm_prefix_,
        namespace_,
        xyz_,
        index_):

    # robot_ip_str = context.perform_substitution(robot_ip)
    # arm_id_str = context.perform_substitution(arm_id)
    # arm_prefix_str = context.perform_substitution(arm_prefix)
    # use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    # fake_sensor_commands_str = context.perform_substitution(
    #     fake_sensor_commands)
    # load_gripper_str = context.perform_substitution(load_gripper)

    franka_xacro_filepath = os.path.join(get_package_share_directory(
        'franka_robotiq'), 'urdf', 'fr3_robotiq.urdf.xacro')
    robot_description = xacro.process_file(franka_xacro_filepath,
                                           mappings={
                                               'ros2_control': 'true',
                                               'arm_id': arm_id_,
                                               'robot_ip': robot_ip_,
                                               'hand': load_gripper_,
                                               'use_fake_hardware': use_fake_hardware_,
                                               'fake_sensor_commands': fake_sensor_commands_,
                                               'arm_prefix': arm_prefix_,
                                               'xyz': xyz_,
                                               'index': index_,
                                               'hand': 'true',
                                               'ee_id': 'robotiq',
                                               'gazebo': 'true',
                                               'gazebo_effort': 'false'
                                           }).toprettyxml(indent='  ')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=namespace_,
            parameters=[{'robot_description': robot_description}],
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

    left_robot_ip=''; left_arm_id='fr3'; left_use_fake_hardware='false'; left_fake_sensor_commands='true'; left_load_gripper='true'; left_arm_prefix='left'; left_namespace='left'; left_xyz= '0 0.4 0'; left_index='0'
    
    robot_state_publisher_left = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            left_robot_ip,
            left_arm_id,
            left_use_fake_hardware,
            left_fake_sensor_commands,
            left_load_gripper,
            left_arm_prefix,
            left_namespace,
            left_xyz,
            left_index])
    
    right_robot_ip=''; right_arm_id='fr3'; right_use_fake_hardware='true'; right_fake_sensor_commands='true'; right_load_gripper='true'; right_arm_prefix='right'; right_namespace='right'; right_xyz= '0 -0.4 0'; right_index='1'
    robot_state_publisher_right = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            right_robot_ip,
            right_arm_id,
            right_use_fake_hardware,
            right_fake_sensor_commands,
            right_load_gripper,
            right_arm_prefix,
            right_namespace,
            right_xyz,
            right_index])
    pkg_my_sim = get_package_share_directory('franka_robotiq')
    world_path = os.path.join(pkg_my_sim, 'worlds', 'gravity_zero.sdf')

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
    spawn_left = Node(
        package='ros_gz_sim',
        executable='create',
        namespace="left",
        arguments=['-topic', '/left/robot_description'],
        output='screen',
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    joint_gui=Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            namespace='left',
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', '--controller-manager', '/left/controller_manager',
                'joint_state_broadcaster'],
        output='screen'
    )

    left_cartesian_motion_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', '--controller-manager', '/left/controller_manager',
                'left_cartesian_motion_controller'],
        output='screen',
    )

    left_robotiq_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', '--controller-manager', '/left/controller_manager',
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
        robot_state_publisher_left,
        rviz,
        spawn_left,
        #joint_gui,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_left,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[left_cartesian_motion_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=left_cartesian_motion_controller,
        #         on_exit=[right_cartesian_motion_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=left_cartesian_motion_controller,
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
