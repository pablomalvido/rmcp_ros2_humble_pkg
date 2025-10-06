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
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
from launch_ros.parameter_descriptions import ParameterValue


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


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print("ERROR")
        return None
    

def generate_launch_description():

    # planning_context
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_robotiq'),
        'urdf',
        'multi_arm.urdf.xacro'
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
            ' ros2_control:=true',
            ' hand:=false', 
            ' ee_id:=robotiq',
            ' gazebo:=true',
            ' effort_gazebo:=false',
            ' hand:=true',
            ' use_fake_hardware:=false',
            ' number_of_robots:=2'])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_robotiq_moveit_config'),
        'config',
        'multi_fr3.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    kinematics_yaml = load_yaml(
        'franka_robotiq_moveit_config', 'config/multi_kinematics.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_robotiq_moveit_config', 'config/multi_ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_robotiq_moveit_config', 'config/multi_custom_moveit_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace='',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'franka_robotiq_moveit_config'), 'config')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )
    # joint_controllers_file = os.path.join(
    #     get_package_share_directory('ur_yt_sim'), 'config', 'ur5_controllers_gripper.yaml'
    # )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='',
        output='both',
        parameters=[robot_description],
    )

    ########################################

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

    left_cartesian_motion_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'left_cartesian_motion_controller'],
        output='screen'
    )

    right_cartesian_motion_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'right_cartesian_motion_controller'],
        output='screen'
    )

    joint_position_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'joint_position_example_controller'],
        output='screen'
    )

    left_moveit_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'left_robotiq_gripper_controller'],
        output='screen'
    )

    left_moveit_franka_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'left_joint_trajectory_controller'],
        output='screen'
    )

    right_moveit_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'right_robotiq_gripper_controller'],
        output='screen'
    )

    right_moveit_franka_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'right_joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_empty_world,
        robot_state_publisher,
        rviz_node,
        spawn,
        #move_group_node,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[move_group_node],
                )
        ),
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[left_moveit_franka_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[left_moveit_gripper_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[right_moveit_franka_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[right_moveit_gripper_controller],
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
                target_action=load_joint_state_broadcaster,
                on_exit=[right_cartesian_motion_controller],
            )
        ),
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
