---
name: new-service-scaffold
description: Load when creating a new service package, to get the file list, CI template and container wiring.
---
Rules are in AGENTS.md "Microservice rules" and "Package names"; confirm the package name with the
owner first. Replace `<Name>` / `<name>` below.

## Files
```
<Name>/package.xml            real description + maintainer (the owner), depend / exec_depend, pip deps as XML comment
<Name>/README.md              topics table, parameters table, run and test commands (shape: Perception/README.md)
<Name>/launch/<name>.launch.py   installed (Python: setup.py data_files; C++: install(DIRECTORY launch ...))
<Name>/test/                  logic tests + node tests (skill: unit-tests)
Python:  setup.py, setup.cfg, resource/<name> (empty), <name>/__init__.py, <name>/<node>.py
C++:     CMakeLists.txt (ament_cmake_gtest for tests), include/<name>/*.hpp, src/*.cpp
<name>_msgs/                  package.xml, CMakeLists.txt, msg/<Type>.msg; no src/ or include/
.github/workflows/<name>-tests.yml
```
Message rules (header first, units and frame on every field) are in AGENTS.md "Messages". Existing
examples: `perception_msgs/msg/Detection.msg`, `tracking_msgs/msg/TrackingState.msg`.

## CI
Copy `.github/workflows/perception-tests.yml`. Swap the two `paths:` lists to `<Name>/**`,
`<name>_msgs/**` and the workflow's own path, the package names in both `colcon` steps, and the
`--base-paths`. Keep the plain `ros:jazzy` container. C++ services that link MAVSDK also install the
`libmavsdk-dev` .deb (see `.github/workflows/movement-tests.yml`).

## `.devcontainer/post-create.sh`
```bash
ln -sfn /workspace/<name>_msgs /opt/ros2_ws/src/<name>_msgs
ln -sfn /workspace/<Name> /opt/ros2_ws/src/<Name>
# add <name>_msgs <name> to the colcon build --packages-select list
echo "alias <name>='ros2 launch <name> <name>.launch.py'" >> /etc/bash.bashrc   # one alias, delete any old one first
```

## README tables
`| Direction | Topic | Type | Notes |` for the interface and `| Name | Default | Meaning |` for
parameters, then Run and Test blocks.
