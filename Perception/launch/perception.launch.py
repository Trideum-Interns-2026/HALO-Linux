from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        # The gazebo_ros camera plugin publishes under
        # <namespace>/<camera_name>/image_raw — with namespace "/camera" and
        # camera_name "front" that's /camera/front/image_raw, not
        # /camera/image_raw.
        default_value="/camera/front/image_raw",
        description="Camera image topic to subscribe to",
    )

    return LaunchDescription(
        [
            image_topic_arg,
            Node(
                package="perception",
                executable="red_sphere_detector",
                name="red_sphere_detector",
                output="screen",
                parameters=[
                    {"image_topic": LaunchConfiguration("image_topic")},
                    {"min_contour_area": 150.0},
                    {"publish_debug_image": True},
                ],
            ),
        ]
    )