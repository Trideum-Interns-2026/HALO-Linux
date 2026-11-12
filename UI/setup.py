from setuptools import find_packages, setup

package_name = "ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/ui.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cohen Hill",
    maintainer_email="cohen.hill@ascte.org",
    description="Bridge from ROS position topics to the ground-station web API.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["ui_bridge = ui.ui_bridge_node:main"]},
)
