"""打开比赛世界。默认横向整排推送、视觉复查和局部清理，持续监视回推。

  ros2 launch sweep_robot sim.launch.py

只要仿真、自己键盘遥控时:
  ros2 launch sweep_robot sim.launch.py push:=false

复现旧路线用于对照：controller:=follow_path
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
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


def _upright_plugin_dirs() -> list:
    here = os.path.abspath(__file__)
    candidates = [
        os.path.abspath(os.path.join(here, "..", "..", "..", "..", "..", "sweep_robot_gazebo", "lib")),
        "/home/cjr/robot_competition/install/sweep_robot_gazebo/lib",
    ]
    return [path for path in candidates if os.path.isdir(path)]


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
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("controller", default_value="adaptive_controller"),
            DeclareLaunchArgument("sweep_rows", default_value="true"),
            DeclareLaunchArgument("show_window", default_value="true"),
            DeclareLaunchArgument("gazebo_params", default_value=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"config","gazebo.yaml")),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", _model_path(models)),
            SetEnvironmentVariable("LP_NUM_THREADS","1"),
            SetEnvironmentVariable(
                "GAZEBO_PLUGIN_PATH",
                _prepend_env("GAZEBO_PLUGIN_PATH", _upright_plugin_dirs() + ros_libs),
            ),
            SetEnvironmentVariable("LD_LIBRARY_PATH", _prepend_env("LD_LIBRARY_PATH", ros_libs)),
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "--lockstep",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    LaunchConfiguration("world"),
                    "--ros-args", "--params-file", LaunchConfiguration("gazebo_params"), "--",
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    "--verbose",
                    "--lockstep",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    LaunchConfiguration("world"),
                    "--ros-args", "--params-file", LaunchConfiguration("gazebo_params"), "--",
                ],
                condition=UnlessCondition(LaunchConfiguration("gui")),
                output="screen",
            ),
            Node(
                package="sweep_robot",
                executable="hsv_detector",
                name="hsv_detector",
                output="screen",
                parameters=[{"use_sim_time": True, "show_window": LaunchConfiguration("show_window"),
                             "reliable_images": True,
                             "camera_height": 2.55}],
            ),
            Node(
                package="sweep_robot",
                executable=LaunchConfiguration("controller"),
                name="follow_path",
                output="screen",
                condition=IfCondition(LaunchConfiguration("push")),
                parameters=[
                    {"use_sim_time": True, "autostart": LaunchConfiguration("autostart"),
                     "sweep_rows": LaunchConfiguration("sweep_rows")},
                ],
            ),
        ]
    )
