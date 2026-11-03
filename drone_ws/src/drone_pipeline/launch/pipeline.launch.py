from launch import LaunchDescription
from launch_ros.actions import Node

NODES = [
    "world_sim", "camera_sim", "px4_sim",            # fake inputs
    "perception_node", "geolocation_node",            # the real pipeline
    "poi_tracker_node", "movement_node", "web_node",
]


def generate_launch_description():
    return LaunchDescription([
        Node(package="drone_pipeline", executable=n, name=n, output="screen", parameters=([{"port": 8081}] if n == "web_node" else []))
        for n in NODES
    ])
