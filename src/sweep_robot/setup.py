from setuptools import setup
import os
from glob import glob

package_name = "sweep_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@todo.todo",
    description="Sweep robot nodes for Gazebo and Jetson Nano",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teleop = sweep_robot.teleop_keyboard:main",
            "mecanum_ik = sweep_robot.mecanum_ik:main",
            "hsv_detector = sweep_robot.hsv_detector:main",
            "push_blocks = sweep_robot.push_blocks:main",
            "record_trajectory = sweep_robot.record_trajectory:main",
            "follow_path = sweep_robot.follow_path:main",
            "adaptive_controller = sweep_robot.adaptive_controller:main",
        ],
    },
)
