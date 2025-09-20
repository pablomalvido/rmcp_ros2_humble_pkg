from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    
    publisher_node = Node(
        package='topics_pkg',
        executable='talker',
        name='talker_node',
        output='screen'
    )
    subscriber_node = Node(
        package='topics_pkg',
        executable='listener',
        name='listener_node',
        output='screen',
        emulate_tty=True  # <-- helpful to force flushing prints
    )
    return LaunchDescription(
        [publisher_node,
        subscriber_node]
    )

