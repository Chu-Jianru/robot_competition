# 比赛清扫机器人

Windows 项目根目录为 `E:\robot_competition`；原 Ubuntu 根目录 `/home/cjr/robot_competition` 是 ROS2 Humble 的 colcon 工作空间。功能包是 `src/sweep_robot`，另保留 `src/sweep_robot_gazebo` 插件包。Gazebo 场地和车在 `competition_robot/`，由 `competition_robot/models/competition_robot/build_robot.py` 生成。改场地或车之后运行这个脚本，不要手改 `competition_robot/worlds/competition.world`。

仓库总览先看 [`README.md`](README.md)，文档索引在 [`docs/README.md`](docs/README.md)，当前状态只认 [`交接说明.md`](交接说明.md)。接手时先确认当前是 Ubuntu 还是 Windows，不把历史结果或拟定目录当实机验收。

## 当前主线与环境

Windows 实机链路是当前主线：手机俯视取图 → USB/scrcpy 5.0.1 → Windows Python/OpenCV → 蓝牙 SPP → ESP32 麦轮底盘。比赛当天不跑 ROS、不跑 Gazebo。Windows 实机不运行 ROS、Gazebo 或 colcon，直接复用纯视觉/策略模块；不要并行开两个摄像头客户端。

Windows 用 Python 3.12.2。项目内 `.deps/`、`.deps-gpu/` 是训练/推理依赖，`tools/*-venv/`、`tools/pio-*` 是面板和 PlatformIO 环境，都只保留在本机，已在 `.gitignore` 排除。完整 scrcpy 和已有 `.venv` 在 `E:\scrcpy\scrcpy-win64-v5.0.1`，从项目启动时传 `--scrcpy-dir`。

现场标定、整车闭环和完整比赛流程**尚未验收**；`log/` 里的历史结果只是开发证据，不是实机通过。h15 仿真已按使用方要求暂停，最终矩阵未完成；整理文档或开发 Windows 不代表获准恢复仿真。未获准不要启动仿真、相机、串口、电机、烧录或演示。

训练只在本机跑，不在 DSH 沙箱里跑（安装和训练需要写 `E:\scrcpy` 下的 venv）。本机有 NVIDIA RTX 4060 Laptop 8 GB，可本地训练；`training/runs/`、`training/*.pt`、`training/*.zip` 都不入库。

## 仓库结构

| 路径 | 内容 |
|---|---|
| `windows/` | 实机客户端：取图、视觉、标定、任务、诊断、离线测试；`test_data/` 数据，`models/` ONNX，`tags/` 图纸 |
| `src/sweep_robot/` | ROS2 包：纯 Python 策略、仿真适配、节点；`test/` 离线测试 |
| `competition_robot/` | Gazebo 场地、车体模型和生成脚本 |
| `training/` | YOLO 训练脚本、Colab 笔记本、本机权重 |
| `hardware/` | 固件源码、烧录说明、接线表；封存工作台 `workbench/` 只在本机 |
| `vendor/` | 队友 2026-11-02 的 WHEEL_SPEED_V1 源码包，只读 |
| `tools/` | 双击入口、面板、PlatformIO 工具链 |
| `docs/` | 设计、命令、盘点、历史汇报和媒体 |
| `log/`、`.deps*`、`tools/pio-*`、`hardware/workbench/` | 本地运行环境/证据，不入库 |

`vendor/` 存放他人提供的源码包（当前为队友 2026-11-02 的 WHEEL_SPEED_V1，含轮速闭环），只读参考：不改内容、不拆目录，否则其 SHA 清单和相对路径失效；要合并进现用固件须先单独确认。

现用可烧录固件副本在 `hardware/firmware/WHEEL_SPEED_V1/motor-bringup/`；2026-10-22 封存的完整工作台在 `hardware/workbench/`，约 184 MB，只在本机运行、不入库，界面源码可对照 `vendor/`。接线资料在 `hardware/`。

## 命令

Ubuntu 新终端会通过 `~/.bashrc` 加载 `/opt/ros/humble/setup.bash` 和本仓库的 `install/setup.bash`。不要改 bashrc 里的 `cmu_exploration_ws`。Windows PowerShell 不使用这套启动环境。

只有修改 Ubuntu ROS 代码并获准运行仿真时，才使用以下编译/启动流程；仅改文档不需要启动仿真：

```bash
cd /home/cjr/robot_competition
colcon build --packages-select sweep_robot
ros2 launch sweep_robot sim.launch.py
```

`ros2 launch` 会同时打开 Gazebo、俯视检测和推块。只要键盘遥控时加 `push:=false`。

默认推块入口为 `adaptive_controller`（节点名仍是 `follow_path`），整排任务在 `row_policy.py`，局部清理及公共安全检查在 `competition_policy.py`。旧路线对照用 `controller:=follow_path`。

Windows 日常入口是双击 [`tools/日常工具.cmd`](tools/日常工具.cmd)；它默认不连接设备。详细命令见 [`docs/常用命令.md`](docs/常用命令.md)。

## 比赛

场地 1.2m×1.2m，蓝黄各半，围栏高 10cm。己方是蓝区，坐标 y<0。约 2cm 的红块共 20 个，中线两侧各 10 个。车从蓝区底边出发，车头朝黄区。90 秒内己方红块更少的一方获胜。比赛中无人干预，和车之间无线连接。

## 视觉和推块

己方按红块周围的地面颜色判断：蓝多于黄才算己方。不要把场地四边形的几何中心当成中线。

红块检测：**HSV 是稳定基线**，始终保留回退。2026-11-10 已训练出 `windows/models/red_block.onnx`；`run_task` 的 `--detector auto` 会在模型存在时优先用 ONNX，缺模型时回退 HSV。是否把 ONNX 定为比赛默认，要看 `windows/evaluate_detector.py` 的对比结果，不能只凭训练集 mAP 宣称已超过 HSV。

车顶 tag：`DICT_4X4_50`、50 mm、**ID 7**。唯一默认值是 `windows/tag_detector.TAG_ID`，台架、位姿、任务、诊断入口和图纸都从它取；换 ID 只改这一处，再重跑图纸脚本。车上不要同时贴两枚同 ID。

取流默认 **1280 档**；旧 1920 档实测会停帧。预览与识别分频尚未实施，不因此擅自改变仿真相机或放宽场景过期阈值。

现有策略为横向整排推送：车头先在空区转向东，连续推对方排、己方排，再在空区转向北、沿墙清理。允许按当前视觉跳过受阻或已散开的整排任务，切换北向局部清理。横推必须由车头推送，不能让车身侧后部扫块；转向前检查净空。仿真相机和默认感知保持 640×480、10Hz。「俯视相机」窗口只供观看，不要为了显示再渲染一张大图。

未要求时不要加对手机器人。

## 上场

比赛当天不跑虚拟机，也不跑 Gazebo。Windows 上用 Python 和 OpenCV 调用视觉模块，把 `vx vy wz` 经无线发给单片机。现场标定和整车闭环完成前，不得把离线结果当成比赛能力。
