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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
                                               'ee_id': 'robotiq',
                                               'gazebo': 'true',
                                               'gazebo_effort': 'false',
                                               'ns': '/'+namespace_
                                           }).toprettyxml(indent='  ')
    
    franka_controllers = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml']),

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=namespace_,
            parameters=[{'robot_description': robot_description}],
        ),

        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     namespace=namespace_,
        #     parameters=[franka_controllers,
        #                 {'robot_description': robot_description},
        #                 {'arm_id': arm_id_},
        #                 {'load_gripper': load_gripper_},
        #                 ],
        #     output={
        #         'stdout': 'screen',
        #         'stderr': 'screen',
        #     },
        #     on_exit=Shutdown(),
        # ),
        
        ]


def generate_launch_description():
    nodes=[]
    #rviz_file = os.path.join(get_package_share_directory('franka_robotiq'), 'rviz', 'visualize_franka.rviz')

    left_robot_ip=''; left_arm_id='fr3'; left_use_fake_hardware='false'; left_fake_sensor_commands='true'; left_load_gripper='false'; left_arm_prefix='left'; left_namespace='NS_1'; left_xyz= '0 0.4 0'; left_index='0'
    
    nodes.append(OpaqueFunction(
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
            left_index]))
    
    right_robot_ip=''; right_arm_id='fr3'; right_use_fake_hardware='false'; right_fake_sensor_commands='true'; right_load_gripper='false'; right_arm_prefix='right'; right_namespace='NS_2'; right_xyz= '0 -0.4 0'; right_index='1'
    nodes.append(OpaqueFunction(
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
            right_index]))
        
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', PathJoinSubstitution([
            FindPackageShare('franka_robotiq'), 'rviz', 'visualize_franka_dual.rviz'
        ])],
        output='screen',
    ))

    # GAZEBO WORLD
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        #launch_arguments={'gz_args': f'{world_path}'+' -r', }.items(),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    ))

    left_namespace_modified = ""
    if len(left_namespace)>0:
        left_namespace_modified='/'+left_namespace
    # Spawn
    spawn1 = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=left_namespace,
        arguments=['-topic', left_namespace_modified+'/robot_description'],
        output='screen',
    )

    right_namespace_modified = ""
    if len(right_namespace)>0:
        right_namespace_modified='/'+right_namespace
    # Spawn
    spawn2 = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=right_namespace,
        arguments=['-topic', right_namespace_modified+'/robot_description'],
        output='screen',
    )

    joint_state_broadcaster1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=left_namespace,
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    joint_state_broadcaster2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=right_namespace,
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    cartesian_controller1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=left_namespace,
        arguments=["left_cartesian_motion_controller", '--controller-manager-timeout', '30'],
        output='screen',
    )

    cartesian_controller2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=right_namespace,
        arguments=["right_cartesian_motion_controller", '--controller-manager-timeout', '30'],
        output='screen',
    )

    # nodes.append(Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     namespace=left_namespace,
    #     arguments=["left_cartesian_motion_controller", '--controller-manager-timeout', '30'],
    #     parameters=[PathJoinSubstitution([
    #         FindPackageShare('franka_bringup'), 'config', "controllers.yaml",
    #     ])],
    #     output='screen',
    # ))

    # nodes.append(Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     namespace=right_namespace,
    #     arguments=["right_cartesian_motion_controller", '--controller-manager-timeout', '30'],
    #     parameters=[PathJoinSubstitution([
    #         FindPackageShare('franka_bringup'), 'config', "controllers.yaml",
    #     ])],
    #     output='screen',
    # ))

    nodes.append(spawn1)

    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn1,
            on_exit=[joint_state_broadcaster1]
        )
    ))

    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster1,
            on_exit=[cartesian_controller1]
        )
    ))

    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn1,
            on_exit=[TimerAction(
                period=2.0,  # optional delay (seconds) before spawning the second robot
                actions=[spawn2]
            )]
        )
    ))

    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn2,
            on_exit=[joint_state_broadcaster2]
        )
    ))

    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster2,
            on_exit=[cartesian_controller2]
        )
    ))

    return LaunchDescription(nodes)
