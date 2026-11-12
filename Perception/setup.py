from setuptools import find_packages, setup

package_name = "perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Marker file so ROS can find the package by name
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Launch files, so `ros2 launch perception ...` can find them
        ("share/" + package_name + "/launch", [
            "launch/gz_harmonic_bridge.launch.py",
            "launch/gz_gps_bridge.launch.py",
            "launch/perception.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cohen",
    maintainer_email="cohen.hill@ascte.org",
    description="YOLO person detector: camera images in, Detection messages out.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_person_detector = perception.yolo_person_detector:main",
            "detection_overlay = perception.detection_overlay:main",
        ],
    },
)