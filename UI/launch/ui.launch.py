"""Start the UI bridge node. In the simulator pass use_sim_time:=true (see scripts/sim_links):
position stamps are Gazebo sim time. Leave it false on real hardware.
api_url:=http://HOST:8000/api/telemetry points the bridge at a web API that is not on localhost
(for example in another container)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("api_url", default_value="http://127.0.0.1:8000/api/telemetry"),
        Node(
            package="ui",
            executable="ui_bridge",
            name="ui_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "api_url": LaunchConfiguration("api_url"),
            }],
        ),
    ])
