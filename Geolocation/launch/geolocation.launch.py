from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="geolocation",
            executable="geolocation_node",
            name="geolocation",
            output="screen",
        )
    ])
