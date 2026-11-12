"""Bridges the drone's simulated GPS (-> /gps/fix) and the sim clock (-> /clock) to ROS 2.

Sim-only tooling, like gz_harmonic_bridge.launch.py (the same auto-detect idea): Gazebo names
the navsat topic after the world and model, so this finds it in whatever world is running.
Geolocation reads /gps/fix (sensor_msgs/NavSatFix). The camera and GPS stamp their messages with
Gazebo sim time, so nodes that compare stamps with their own clock (Tracking) must run with
use_sim_time:=true; /clock is what makes that work. Pass gz_navsat_topic:=... to pick one.
"""

import re
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROS_GPS_TOPIC = "/gps/fix"
GZ_NAVSAT_TOPIC = re.compile(r"^/world/[^/]+/model/.+/sensor/[^/]+/navsat$")
DISCOVERY_TIMEOUT_S = 15.0


def find_gz_navsat_topic():
    deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
    while True:
        topics = subprocess.run(
            ["gz", "topic", "-l"], capture_output=True, text=True
        ).stdout.split()
        found = sorted(t for t in topics if GZ_NAVSAT_TOPIC.match(t))
        if found:
            return found[0]
        if time.monotonic() > deadline:
            raise RuntimeError("No Gazebo navsat topic found — is `fly` running?")
        time.sleep(1.0)


def make_bridge(context):
    gz_topic = LaunchConfiguration("gz_navsat_topic").perform(context) or find_gz_navsat_topic()
    print(f"Bridging Gazebo GPS '{gz_topic}' -> '{ROS_GPS_TOPIC}'")
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_gps_bridge",
            output="screen",
            arguments=[
                f"{gz_topic}@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
            remappings=[(gz_topic, ROS_GPS_TOPIC)],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gz_navsat_topic", default_value="",
                              description="Gazebo navsat topic to bridge (default: auto-detect)"),
        OpaqueFunction(function=make_bridge),
    ])
