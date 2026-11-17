"""Start the geolocation node. params_file:=<yaml> loads a parameter file such as
config/tuned.yaml (see README, "Tuning")."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    return [Node(package="geolocation", executable="geolocation_node", name="geolocation",
                 output="screen", parameters=[params_file] if params_file else [])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value="",
                              description="YAML parameter file, e.g. config/tuned.yaml"),
        OpaqueFunction(function=make_node),
    ])
