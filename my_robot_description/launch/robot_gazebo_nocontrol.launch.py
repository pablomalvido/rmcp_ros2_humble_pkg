import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # LOAD ROBOT DESCRIPTION (URDF)
    robot_xacro_filepath = os.path.join(get_package_share_directory(
            'my_robot_description'), 'urdf', 'robot.urdf.xacro')

    robot_description = xacro.process_file(
        robot_xacro_filepath,
        mappings={
            'ros2_control': 'false',
            'gripper': 'false'
            ### Examples of typical arguments below:
            #'robot_ip': 192.168.0.1,
            #'real_robot': 'true'
            #'arm_prefix': 'right_'
        }
    ).toprettyxml(indent='  ')

    # ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # JOINT STATE PUBLISHER GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # LAUNCH GAZEBO WORLD
    # os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('my_robot_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # SPAWN ROBOT IN THE GAZEBO WORLD
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # RVIZ VISUALIZATION ENVIRONMENT
    rviz_file = os.path.join(get_package_share_directory(
        'my_robot_description'), 'rviz', 'visualize_robot.rviz')
    rviz = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
    )

    return LaunchDescription(
        [
         gazebo_empty_world,
         robot_state_publisher,
         joint_state_publisher_gui,
         rviz,
         spawn
        ]
    )