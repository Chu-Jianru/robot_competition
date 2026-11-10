# 比赛清扫机器人

Windows 项目根目录为 `E:\robot_competition`；原 Ubuntu 根目录 `/home/cjr/robot_competition` 为 ROS 2 Humble 的 colcon 工作空间。功能包为 `src/sweep_robot`，另保留 `src/sweep_robot_gazebo` 插件包。Gazebo 场地和车体位于 `competition_robot/`，由 `competition_robot/models/competition_robot/build_robot.py` 生成；修改场地或车体后运行该脚本，不得手改 `competition_robot/worlds/competition.world`。

仓库总览见 [`README.md`](README.md)，文档索引见 [`docs/README.md`](docs/README.md)，当前状态以 [`交接说明.md`](交接说明.md) 为准。接手时确认当前平台（Ubuntu 或 Windows）；历史结果和拟定目录不作为实机验收依据。

## 当前主线与环境

Windows 实机链路为当前主线：手机俯视取图 → USB/scrcpy 5.0.1 → Windows Python/OpenCV → 蓝牙 SPP → ESP32 麦轮底盘。比赛期间不运行 ROS、Gazebo 或 colcon。Windows 实机直接复用纯视觉/策略模块，不得并行运行两个摄像头客户端。

Windows 使用 Python 3.12.2。项目内 `.deps/`、`.deps-gpu/` 为训练/推理依赖，`tools/*-venv/`、`tools/pio-*` 为面板与 PlatformIO 环境，均仅保留本机，已由 `.gitignore` 排除。完整 scrcpy 与已有 `.venv` 位于 `E:\scrcpy\scrcpy-win64-v5.0.1`，启动时传入 `--scrcpy-dir`。

现场标定、整车闭环和完整比赛流程尚未验收；`log/` 中的历史结果为开发证据，不作为实机通过依据。h15 仿真已按使用方要求暂停，最终矩阵未完成；整理文档或开发 Windows 不构成恢复仿真的许可。未获准时不得启动仿真、相机、串口、电机、烧录或演示。

训练仅在本机执行，不在 DSH 沙箱内运行（安装和训练需写入 `E:\scrcpy` 下的 venv）。本机 GPU 为 NVIDIA RTX 4060 Laptop 8 GB；`training/runs/`、`training/*.pt`、`training/*.zip` 均不入库。

## 仓库结构

| 路径 | 内容 |
|---|---|
| `windows/` | 实机客户端：取图、视觉、标定、任务、诊断、离线测试；`test_data/` 数据，`models/` ONNX，`tags/` 图纸 |
| `src/sweep_robot/` | ROS 2 包：纯 Python 策略、仿真适配、节点；`test/` 离线测试 |
| `competition_robot/` | Gazebo 场地、车体模型和生成脚本 |
| `training/` | YOLO 训练脚本、Colab 笔记本、本机权重 |
| `hardware/` | 固件源码、烧录说明、接线表；封存工作台 `workbench/` 仅在本机 |
| `vendor/` | 队友 2026-11-02 的 WHEEL_SPEED_V1 源码包，只读 |
| `tools/` | 双击入口、面板、PlatformIO 工具链 |
| `docs/` | 设计、命令、盘点、历史汇报和媒体 |
| `log/`、`.deps*`、`tools/pio-*`、`hardware/workbench/` | 本地运行环境或证据，不入库 |

`vendor/` 为他人提供的源码包（当前为队友 2026-11-02 的 WHEEL_SPEED_V1，含轮速闭环），仅作参考：不得修改内容或拆分目录，否则其 SHA 清单和相对路径失效；合并进现用固件前需单独确认。

现用可烧录固件副本位于 `hardware/firmware/WHEEL_SPEED_V1/motor-bringup/`；2026-10-22 封存的完整工作台位于 `hardware/workbench/`，约 184 MB，仅在本机运行、不入库，界面源码可对照 `vendor/`。接线资料位于 `hardware/`。

## 命令

Ubuntu 新终端通过 `~/.bashrc` 加载 `/opt/ros/humble/setup.bash` 和本仓库的 `install/setup.bash`；不得修改 bashrc 中的 `cmu_exploration_ws`。Windows PowerShell 不使用该启动环境。

仅在修改 Ubuntu ROS 代码并获准运行仿真时，使用以下编译与启动流程；仅修改文档无需启动仿真：

```bash
cd /home/cjr/robot_competition
colcon build --packages-select sweep_robot
ros2 launch sweep_robot sim.launch.py
```

`ros2 launch` 同时打开 Gazebo、俯视检测和推块；仅键盘遥控时加 `push:=false`。

默认推块入口为 `adaptive_controller`（节点名仍为 `follow_path`），整排任务在 `row_policy.py`，局部清理及公共安全检查在 `competition_policy.py`；旧路线对照使用 `controller:=follow_path`。

Windows 日常入口为 [`tools/日常工具.cmd`](tools/日常工具.cmd)，默认不连接设备；详细命令见 [`docs/常用命令.md`](docs/常用命令.md)。

## 比赛

场地 1.2 m × 1.2 m，蓝黄各半，围栏高 10 cm。己方为蓝区，坐标 y<0。红块约 2 cm，共 20 个，中线两侧各 10 个。车从蓝区底边出发，车头朝黄区。90 秒内己方红块更少的一方获胜。比赛期间无人干预，车与操作端之间为无线连接。

## 视觉和推块

己方按红块周围的地面颜色判断：蓝多于黄才算己方。不得将场地四边形的几何中心作为中线。

红块检测以 HSV 为基线，并保留回退。2026-11-10 已训练 `windows/models/red_block.onnx`；`run_task` 的 `--detector auto` 在模型存在时优先使用 ONNX，缺失时回退 HSV。是否将 ONNX 作为比赛默认，需依据 `windows/evaluate_detector.py` 的对比结果，不能仅凭训练集 mAP 判定。

车顶 tag：`DICT_4X4_50`、50 mm、ID 7。唯一默认值为 `windows/tag_detector.TAG_ID`，台架、位姿、任务、诊断入口和图纸均从该值读取；修改 ID 后需重新生成图纸。车上不得同时贴两枚同 ID。

取流默认 1280 档；1920 档实测存在停帧。预览与识别分频尚未实施，不得因此修改仿真相机或放宽场景过期阈值。

现有策略为横向整排推送：车头先在空区转向东，连续推对方排、己方排，再在空区转向北、沿墙清理。允许按当前视觉跳过受阻或已散开的整排任务，切换北向局部清理。横推必须由车头推送，不得由车身侧后部扫块；转向前检查净空。仿真相机和默认感知保持 640×480、10 Hz。「俯视相机」窗口仅用于观看，不得为显示再渲染一张大图。

未要求时不添加对手机器人。

## 上场

比赛期间不运行虚拟机或 Gazebo。Windows 上使用 Python 和 OpenCV 调用视觉模块，将 `vx vy wz` 经无线发送至单片机。现场标定和整车闭环完成前，离线结果不作为比赛能力。
