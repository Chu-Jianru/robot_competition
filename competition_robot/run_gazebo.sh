#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${ROOT}/worlds/competition.world"

if [ ! -f "${WORLD}" ]; then
  echo "找不到世界文件: ${WORLD}"
  exit 1
fi

# ROS / Gazebo 的 setup 脚本会读尚未赋值的变量，和 set -u 冲突
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/foxy/setup.bash
else
  echo "未找到 ROS。请先安装: sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs"
  exit 1
fi

if [ -f /usr/share/gazebo/setup.sh ]; then
  # shellcheck disable=SC1091
  source /usr/share/gazebo/setup.sh
fi
set -u

# 不沿用当前 shell 里的 GAZEBO_MODEL_PATH。
# 别的工作空间（例如 velodyne）会把 share/ 整目录塞进来，Gazebo 会把里面每个子目录当成模型。
SYSTEM_MODELS="/usr/share/gazebo-11/models"
if [ -d "${SYSTEM_MODELS}" ]; then
  export GAZEBO_MODEL_PATH="${ROOT}/models:${SYSTEM_MODELS}"
else
  export GAZEBO_MODEL_PATH="${ROOT}/models"
fi
export GAZEBO_RESOURCE_PATH="${ROOT}:${ROOT}/worlds:${GAZEBO_RESOURCE_PATH:-}"

if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  IFS=':' read -ra prefixes <<< "${AMENT_PREFIX_PATH}"
  for p in "${prefixes[@]}"; do
    if [ -d "${p}/lib" ]; then
      export GAZEBO_PLUGIN_PATH="${p}/lib:${GAZEBO_PLUGIN_PATH:-}"
      export LD_LIBRARY_PATH="${p}/lib:${LD_LIBRARY_PATH:-}"
    fi
  done
fi

echo "工程目录: ${ROOT}"
echo "世界文件: ${WORLD}"

if ! command -v gazebo >/dev/null 2>&1; then
  echo "未找到 gazebo 命令。请安装: sudo apt install gazebo ros-humble-gazebo-ros-pkgs"
  exit 1
fi

PLUGIN_ARGS=()
plugin_dir=""
IFS=':' read -ra plugin_dirs <<< "${GAZEBO_PLUGIN_PATH:-}"
for d in "${plugin_dirs[@]}"; do
  if [ -f "${d}/libgazebo_ros_init.so" ]; then
    plugin_dir="${d}"
    break
  fi
done

if [ -n "${plugin_dir}" ]; then
  PLUGIN_ARGS=(-s libgazebo_ros_init.so -s libgazebo_ros_factory.so)
  echo "已找到 ROS 插件: ${plugin_dir}"
else
  echo "未找到 libgazebo_ros_init.so，先只开世界。若要 cmd_vel 控制请安装:"
  echo "  sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins"
fi

exec gazebo --verbose "${PLUGIN_ARGS[@]}" "${WORLD}"
