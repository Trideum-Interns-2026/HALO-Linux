#!/usr/bin/env python3
"""Add a ROS 2 camera bridge to the typhoon_h480 Gazebo Classic model.

PX4's typhoon_h480 camera sensor only has plugins for the MAVLink camera
protocol and the GStreamer video stream QGroundControl uses (see
sitl_gazebo-classic/models/typhoon_h480/typhoon_h480.sdf.jinja) — neither
publishes a ROS 2 topic. This script inserts a gazebo_ros camera plugin
into that sensor block, and the gazebo_ros init/factory plugins into the
typhoon_h480 world, so the sim publishes sensor_msgs/Image on
/camera/image_raw for the Perception package to consume.

Idempotent: safe to run on every container start (entrypoint.sh does).
Run after PX4-Autopilot (with its sitl_gazebo-classic submodule) has been
cloned, and before `make px4_sitl` builds/generates the SDF from the
.sdf.jinja template.
"""

import re
import sys
from pathlib import Path

CAMERA_PLUGIN_MARKER = "libgazebo_ros_camera.so"
CAMERA_PLUGIN_BLOCK = """\
        <plugin name="ros_camera_bridge" filename="libgazebo_ros_camera.so">
            <ros>
                <namespace>/camera</namespace>
                <remapping>image_raw:=image_raw</remapping>
                <remapping>camera_info:=camera_info</remapping>
            </ros>
            <camera_name>front</camera_name>
            <frame_name>cgo3_camera_link</frame_name>
        </plugin>
"""

WORLD_PLUGIN_MARKER = "libgazebo_ros_init.so"
WORLD_PLUGIN_BLOCK = """\
    <plugin name="gazebo_ros_init" filename="libgazebo_ros_init.so"/>
    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>
"""

# Matches the <sensor name="camera" type="camera"> ... </sensor> block
# specifically (not the sibling camera_imu sensor).
CAMERA_SENSOR_RE = re.compile(
    r'(<sensor name="camera" type="camera">.*?)(</sensor>)', re.DOTALL
)


def find_sitl_gazebo_root(px4_root: Path) -> Path:
    candidate = px4_root / "Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    if candidate.is_dir():
        return candidate
    raise FileNotFoundError(
        f"sitl_gazebo-classic submodule not found under {px4_root} — "
        "has PX4-Autopilot been cloned with --recurse-submodules?"
    )


def patch_camera_sensor(sdf_jinja_path: Path) -> bool:
    text = sdf_jinja_path.read_text()
    if CAMERA_PLUGIN_MARKER in text:
        return False  # already patched

    match = CAMERA_SENSOR_RE.search(text)
    if not match:
        raise ValueError(
            f"Could not find the camera sensor block in {sdf_jinja_path} — "
            "the model file may have changed upstream; update this script."
        )

    patched = CAMERA_SENSOR_RE.sub(
        lambda m: m.group(1) + CAMERA_PLUGIN_BLOCK + m.group(2), text, count=1
    )
    sdf_jinja_path.write_text(patched)
    return True


def patch_world(world_path: Path) -> bool:
    text = world_path.read_text()
    if WORLD_PLUGIN_MARKER in text:
        return False  # already patched

    if "</world>" not in text:
        raise ValueError(f"Could not find </world> in {world_path}")

    patched = text.replace("</world>", WORLD_PLUGIN_BLOCK + "  </world>", 1)
    world_path.write_text(patched)
    return True


def main():
    px4_root = Path(sys.argv[1] if len(sys.argv) > 1 else "/PX4-Autopilot")
    sitl_gazebo_root = find_sitl_gazebo_root(px4_root)

    sdf_jinja_path = sitl_gazebo_root / "models/typhoon_h480/typhoon_h480.sdf.jinja"
    world_path = sitl_gazebo_root / "worlds/typhoon_h480.world"

    changed = False
    if sdf_jinja_path.is_file():
        if patch_camera_sensor(sdf_jinja_path):
            print(f">> Added ROS camera bridge plugin to {sdf_jinja_path}")
            changed = True
        else:
            print(f">> Camera bridge plugin already present in {sdf_jinja_path}")
    else:
        print(f"!! {sdf_jinja_path} not found, skipping model patch", file=sys.stderr)

    if world_path.is_file():
        if patch_world(world_path):
            print(f">> Added gazebo_ros init/factory plugins to {world_path}")
            changed = True
        else:
            print(f">> gazebo_ros world plugins already present in {world_path}")
    else:
        print(f"!! {world_path} not found, skipping world patch", file=sys.stderr)

    if changed:
        print(">> Camera bridge patch applied — rebuild (make px4_sitl gazebo-classic_typhoon_h480) to regenerate the SDF.")


if __name__ == "__main__":
    main()
