from setuptools import setup

package_name = "perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
        data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/perception.launch.py", "launch/gz_harmonic_bridge.launch.py"],
        ),
        (
            "share/" + package_name + "/config",
            ["config/gz_harmonic_camera_bridge.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cohen",
    maintainer_email="cohen@example.com",
    description="Camera-based red sphere detection for the HALO drone.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "red_sphere_detector = perception.red_sphere_detector:main",
        ],
    },
)
