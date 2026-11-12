"""Start the tracking node. In the simulator pass use_sim_time:=true (needs /clock, see
scripts/sim_links): camera and GPS stamps are Gazebo sim time, which tracking compares with its
own clock. Leave it false on real hardware."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="tracking",
            executable="tracking_node",
            name="tracking",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
