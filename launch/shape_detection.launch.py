from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pennair_2024_shape_detection',
            executable='video_publisher',
            name='video_publisher'
        ),
        Node(
            package='pennair_2024_shape_detection',
            executable='shape_detector',
            name='shape_detector'
        )
    ])