"""Start the camera bridge and the YOLO person detector together.

min_confidence:=<0..1> overrides the detector's confidence threshold (default: the node's own).
overlay:=true also starts detection_overlay, a debug view: /perception/debug_image is the camera
image with the detected box drawn on it (view it with rqt_image_view).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_detector(context):
    parameters = []
    if LaunchConfiguration("min_confidence").perform(context):
        parameters.append({"min_confidence": ParameterValue(
            LaunchConfiguration("min_confidence"), value_type=float)})
    nodes = [Node(package="perception", executable="yolo_person_detector", output="screen",
                  parameters=parameters)]
    if LaunchConfiguration("overlay").perform(context).lower() in ("true", "1", "yes"):
        # Debug view only: the camera image with the detected box drawn on it.
        nodes.append(Node(package="perception", executable="detection_overlay",
                          output="screen"))
    return nodes


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("perception"), "launch")
    return LaunchDescription([
        DeclareLaunchArgument("min_confidence", default_value=""),
        DeclareLaunchArgument("overlay", default_value="false"),
        # Gazebo camera -> /camera/front/image_raw (auto-detects the gz topic)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "gz_harmonic_bridge.launch.py"))),
        # Your node: /camera/front/image_raw -> /perception/detection
        OpaqueFunction(function=make_detector),
    ])
