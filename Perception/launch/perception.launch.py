"""Start the camera bridge and the YOLO person detector together."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("perception"), "launch")
    return LaunchDescription([
        # Gazebo camera -> /camera/front/image_raw (auto-detects the gz topic)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "gz_harmonic_bridge.launch.py"))),
        # Your node: /camera/front/image_raw -> /perception/detection
        Node(package="perception", executable="yolo_person_detector", output="screen"),
    ])