from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='surface_detection',
            executable='simple_2detect',
            name='simple_2detect',
            output='screen'
        ),
        Node(
            package='surface_detection',
            executable='capsule_pointcloud',
            name='capsule_pointcloud',
            output='screen'
        )
    ])
