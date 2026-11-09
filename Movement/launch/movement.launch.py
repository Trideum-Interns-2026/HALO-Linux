"""Start the movement node. Other parameters: ros2 run movement movement_node --ros-args -p ...

orbit:=true circles the person instead of following (see README, "Orbit mode").
params_file:=<yaml> loads a parameter file, e.g. config/sim_tuned.yaml (see README, "Tuning").
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_MAVSDK_URL = "udpin://0.0.0.0:14540"  # SITL; change for HITL / hardware


def make_node(context):
    parameters = []
    params_file = LaunchConfiguration("params_file").perform(context)
    if params_file:
        parameters.append(params_file)   # first, so the arguments below win over the file
    parameters.append({
        "mavsdk_url": LaunchConfiguration("mavsdk_url"),
        "orbit_enabled": ParameterValue(LaunchConfiguration("orbit"), value_type=bool),
        "orbit_period_s": ParameterValue(LaunchConfiguration("orbit_period_s"), value_type=float),
        "orbit_clockwise": ParameterValue(LaunchConfiguration("orbit_clockwise"), value_type=bool),
    })
    return [Node(package="movement", executable="movement_node", output="screen",
                 parameters=parameters)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mavsdk_url", default_value=DEFAULT_MAVSDK_URL),
        DeclareLaunchArgument("orbit", default_value="false"),
        DeclareLaunchArgument("orbit_period_s", default_value="30.0"),
        DeclareLaunchArgument("orbit_clockwise", default_value="false"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="YAML parameter file, e.g. config/sim_tuned.yaml"),
        OpaqueFunction(function=make_node),
    ])
