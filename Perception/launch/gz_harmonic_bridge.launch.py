"""Bridges the drone's color camera from Gazebo Harmonic to ROS 2, onto
/camera/front/image_raw and /camera/front/camera_info.

Gazebo names camera topics after the world and model they're in
(/world/<world>/model/<model>/.../sensor/<camera>/image), so instead of
hardcoding one world's names this finds the camera topic in whatever world
is running when the bridge launches. The ROS-side names stay fixed, so the
detectors never need to know which world they're in.

Pass gz_image_topic:=... to pick a specific camera if a world has several.
"""

import re
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROS_IMAGE_TOPIC = "/camera/front/image_raw"
ROS_CAMERA_INFO_TOPIC = "/camera/front/camera_info"

# Any camera sensor's image topic, in any world/model. The depth camera
# publishes on a bare "depth_camera" topic, so it doesn't match.
GZ_CAMERA_IMAGE_TOPIC = re.compile(r"^/world/[^/]+/model/.+/sensor/[^/]+/image$")

# How long to wait for the sim to come up and advertise a camera.
DISCOVERY_TIMEOUT_S = 15.0


def find_gz_camera_topic():
    deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
    while True:
        topics = subprocess.run(
            ["gz", "topic", "-l"], capture_output=True, text=True
        ).stdout.split()
        cameras = sorted(t for t in topics if GZ_CAMERA_IMAGE_TOPIC.match(t))
        if cameras:
            return cameras[0]
        if time.monotonic() > deadline:
            raise RuntimeError(
                "No Gazebo camera image topic found — is `fly` running with a "
                "camera-equipped model (PX4_SIM_MODEL=gz_x500_depth)?"
            )
        time.sleep(1.0)


def make_bridge(context):
    gz_image_topic = LaunchConfiguration("gz_image_topic").perform(context)
    if not gz_image_topic:
        gz_image_topic = find_gz_camera_topic()
    # camera_info is always the image topic's sibling.
    gz_camera_info_topic = gz_image_topic.rsplit("/", 1)[0] + "/camera_info"
    print(f"Bridging Gazebo camera '{gz_image_topic}' -> '{ROS_IMAGE_TOPIC}'")

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_harmonic_camera_bridge",
            output="screen",
            arguments=[
                f"{gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
                f"{gz_camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                (gz_image_topic, ROS_IMAGE_TOPIC),
                (gz_camera_info_topic, ROS_CAMERA_INFO_TOPIC),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_image_topic",
                default_value="",
                description="Gazebo camera image topic to bridge (default: auto-detect)",
            ),
            OpaqueFunction(function=make_bridge),
        ]
    )
