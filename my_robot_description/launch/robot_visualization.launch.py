import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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
         robot_state_publisher,
         joint_state_publisher_gui,
         rviz
        ]
    )