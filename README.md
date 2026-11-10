# 比赛清扫机器人

场地 1.2 m × 1.2 m，蓝黄各半，围栏高 10 cm；己方为蓝区（y<0），90 秒内将己方红块推至对面，己方剩余红块少者胜。当前实机链路为手机俯视取图 → USB/scrcpy → Windows OpenCV/ONNX → 蓝牙 SPP → ESP32 麦轮底盘。比赛期间不运行 ROS 和 Gazebo。

![实车](docs/media/实车图.jpg)

## 状态

- Windows 实机客户端为当前主线；ROS 2 仿真保留，h15 仿真暂停。
- 现场标定、整车闭环和完整比赛流程尚未验收。
- 当前进度和验证范围以 [交接说明](交接说明.md) 为准；开发约定见 [AGENTS.md](AGENTS.md)。

## 目录

| 路径 | 内容 |
|---|---|
| `windows/` | Windows 实机客户端：取图、视觉、标定、策略接入、诊断、离线测试 |
| `windows/test_data/` | 场地照片、红块训练集、标注与清单；说明见 [数据说明](windows/test_data/README.md) |
| `windows/models/` | 训练后的 `red_block.onnx`，用于车上推理 |
| `windows/tags/` | 车顶 ArUco 图纸（当前 ID 7）与生成脚本 |
| `src/sweep_robot/` | ROS 2 包：纯 Python 策略、仿真适配、节点 |
| `src/sweep_robot_gazebo/` | Gazebo 防翻车插件 |
| `competition_robot/` | Gazebo 场地、车体模型和 `build_robot.py` |
| `training/` | YOLO 训练脚本、Colab 笔记本；权重和运行结果仅在本机 |
| `hardware/` | 固件源码、烧录说明、接线表；封存工作台仅在本机 |
| `vendor/` | 队友 2026-11-02 交付的 WHEEL_SPEED_V1 源码包，只读 |
| `tools/` | 双击入口、面板、PlatformIO 工具链；运行环境仅在本机 |
| `docs/` | 设计、命令、盘点、历史汇报和媒体；说明见 [文档说明](docs/README.md) |
| `records/` | 实车轨迹 CSV |
| `archive/` | 队友原始交付 zip（与 `vendor/` 对应） |

`log/`、`.deps/`、`.deps-gpu/`、`tools/*-venv/`、`tools/pio-*`、`hardware/workbench/` 仅保留在本机，已由 `.gitignore` 排除；`training/runs/`、`training/*.pt`、`training/*.zip` 同样仅保留在本机。

## 常用入口

| 操作 | 位置 |
|---|---|
| 日常菜单 | `tools/日常工具.cmd` |
| 实机自主清扫 | `tools/启动自主清扫.cmd`；前置标定见 [Windows 说明](windows/README.md) |
| 命令与快捷键 | [docs/常用命令.md](docs/常用命令.md) |
| 手机接入与首次配置 | [docs/手机接入与自主清扫手册.md](docs/手机接入与自主清扫手册.md) |
| 红块模型训练 | [training/README.md](training/README.md) |
| 固件与接线 | [hardware/README.md](hardware/README.md)、[hardware/firmware/README.md](hardware/firmware/README.md) |
| ROS 仿真 | [AGENTS.md](AGENTS.md)；h15 已暂停，恢复前需确认 |

## 主要文件

| 任务 | 文件 |
|---|---|
| 取图、红块检测 | `windows/capture_hi.py`、`windows/red_blocks.py` |
| 场地四角标定、车体位姿 | `windows/field_view.py`、`windows/pose_tag.py`、`windows/tag_detector.py` |
| 比赛策略 | `src/sweep_robot/sweep_robot/competition_policy.py`、`row_policy.py`；设计见 [自主策略设计](docs/自主策略设计.md) |
| Windows 任务链路 | `windows/run_task.py`、`windows/nav_servo.py`、`windows/chassis_link.py` |
| 采帧、标注、导出训练集 | `windows/collect_red_frames.py`、`label_boxes.py`、`export_yolo.py` |
| 训练与导出 ONNX | `training/train_local.py`、`windows/训练红块模型.cmd` |
| HSV 与 ONNX 对比 | `windows/evaluate_detector.py`、`windows/比较红块检测.cmd` |
| 固件烧录 | `hardware/firmware/WHEEL_SPEED_V1/motor-bringup/`；步骤见 [tools 说明](tools/README.md) |
| 历史汇报 | [docs/history/](docs/history/)；第三次汇报见 [`docs/history/第三次汇报/`](docs/history/第三次汇报/) |
| 验证证据 | `log/`（仅本机） |

## 配置与限制

- 红块检测：HSV 为基线；训练后的 ONNX 模型位于 `windows/models/red_block.onnx`。`run_task --detector auto` 在模型存在时使用 ONNX，缺失时回退 HSV。ONNX 与 HSV 在同一批人工框上的对比尚未完成。
- 车顶 tag：`DICT_4X4_50`、50 mm、ID 7；ID 定义在 `windows/tag_detector.TAG_ID`，修改后需重新生成图纸。
- 取流：Mate60 Pro + scrcpy 5.0.1，默认 1280 档；1920 档实测存在停帧。
- 验收边界：视觉、标定、策略接入和蓝牙点动已有离线或台架结果；现场标定、整车闭环和比赛全流程尚未验收。`log/` 中的历史结果不作为实机通过依据。
- 仿真：h15 已按使用方要求暂停；当前文档和 Windows 开发不构成恢复仿真的许可。
- `vendor/`：只读，不修改内容或目录结构，否则 SHA 清单和相对路径失效。
- 本地大件：`.deps*`、`tools/pio-*`、`hardware/workbench/`、`log/` 和训练权重已由 `.gitignore` 排除；克隆后需按各 README 准备。
