"""打开比赛世界，并启动俯视检测和推块。

  source /opt/ros/humble/setup.bash
  cd /home/cjr/robot_competition
  colcon build --packages-select sweep_robot
  source install/setup.bash
  ros2 launch sweep_robot sim.launch.py

只要仿真、自己键盘遥控时:
  ros2 launch sweep_robot sim.launch.py push:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _guess_world() -> str:
    here = os.path.abspath(__file__)
    candidates = [
        os.path.join(os.getcwd(), "competition_robot", "worlds", "competition.world"),
        os.path.abspath(
            os.path.join(here, "..", "..", "..", "..", "competition_robot", "worlds", "competition.world")
        ),
        "/home/cjr/robot_competition/competition_robot/worlds/competition.world",
        "/mnt/hgfs/robot_competition/competition_robot/worlds/competition.world",
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


def _model_path(models: str) -> str:
    system_models = "/usr/share/gazebo-11/models"
    if os.path.isdir(system_models):
        return models + ":" + system_models
    return models


def _prepend_env(name: str, entries: list) -> str:
    current = os.environ.get(name, "")
    parts = [item for item in entries if item]
    if current:
        parts.append(current)
    return ":".join(parts)


def _ros_lib_dirs() -> list:
    libs = []
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        lib = os.path.join(prefix, "lib")
        if prefix and os.path.isdir(lib):
            libs.append(lib)
    return libs


def generate_launch_description() -> LaunchDescription:
    default_world = _guess_world()
    world_dir = os.path.dirname(os.path.dirname(default_world))
    models = os.path.join(world_dir, "models")
    ros_libs = _ros_lib_dirs()

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("push", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="false"),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", _model_path(models)),
            SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", _prepend_env("GAZEBO_PLUGIN_PATH", ros_libs)),
            SetEnvironmentVariable("LD_LIBRARY_PATH", _prepend_env("LD_LIBRARY_PATH", ros_libs)),
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    LaunchConfiguration("world"),
                ],
                output="screen",
            ),
            Node(
                package="sweep_robot",
                executable="hsv_detector",
                name="hsv_detector",
                output="screen",
            ),
            Node(
                package="sweep_robot",
                executable="push_blocks",
                name="push_blocks",
                output="screen",
                condition=IfCondition(LaunchConfiguration("push")),
                parameters=[{"autostart": LaunchConfiguration("autostart")}],
            ),
        ]
    )
