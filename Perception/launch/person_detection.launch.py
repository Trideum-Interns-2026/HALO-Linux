from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        # Where gz_harmonic_bridge.launch.py bridges the OakD-Lite's color
        # camera to.
        default_value="/camera/front/image_raw",
        description="Camera image topic to subscribe to",
    )

    return LaunchDescription(
        [
            image_topic_arg,
            Node(
                package="perception",
                executable="person_detector",
                name="person_detector",
                output="screen",
                parameters=[
                    {"image_topic": LaunchConfiguration("image_topic")},
                    {"min_confidence": 0.5},
                    {"detection_width": 960},
                    {"publish_debug_image": True},
                ],
            ),
        ]
    )
