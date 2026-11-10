# 推物块机器人 · 队友源码包

版本：2026-11-02 / **WHEEL_SPEED_V1**。保留开环 MECANUM_V2、FOUR_V1 接线与 CBAD 轮位。此包包含源码、测试和文档，需要在队友电脑安装开发环境；没有附带旧版免安装 EXE。

## 首次运行（Windows）

1. 完整解压到自己的可写目录，保留所有子目录。
2. 准备 64 位 Python 3.12 或以上版本，包含 tkinter，确保 `py` 或 `python` 能运行。
3. 双击根目录 `setup_source.cmd`。脚本会在 `esp32-pc-link/pc/.venv` 建立环境，并按 `requirements.txt` 安装依赖；需要联网。
4. 双击唯一调试入口 `run_debug.cmd`，在窗口顶部选择电机台架、麦克纳姆底盘或整车模拟。端口按队友电脑实际 COM 号选择。
5. 双击 `run_checks.cmd` 做 Python 离线回归；测试使用假串口，不打开真实相机，不运行电机。GUI 测试会短暂创建窗口，需要可用的桌面环境。

本包不携带发送人的个人设置、串口记录、视频源地址或虚拟环境。相机配置使用默认设备 0，不会自动打开相机。ESP32 内已有的 NVS 标定不在源码包中，其他开发板需要单独标定。

## 开发入口

| 目录 | 开发职责 |
|---|---|
| `motor-bringup` | 当前应使用的 ESP32 统一固件，以及台架/底盘 Python 通信接口 |
| `debug-workbench` | 统一调试界面，包含台架、麦轮开环和轮速闭环调参 |
| `esp32-pc-link/pc` | 当前界面依赖的模拟通信、相机、主题与数据路径模块，必须保留 |
| `esp32-pc-link/lib/link_core` | 当前固件依赖的 DRY_RUN 协议核心，必须保留 |
| `esp32-pc-link/src` | 原纯模拟固件参考；正常使用统一固件时不烧录这里 |
| `esp32-connection-test` | 最小 USB/PING 排障参考，不参与正常启动 |
| `docs/superpowers` | 闭环设计与实施计划；实际完成情况以当前代码和发布检查记录为准 |
| `release_tools` | 此源码包的重打包、安装与离线回归工具 |

阅读 [源码与调试指南](motor-bringup/DEBUGGING.md)、[面板说明](debug-workbench/README.md)、[当前接线](motor-bringup/FOUR-MOTOR-WIRING.md) 和 [源码审查记录](docs/整理记录/源码审查与分享说明-2026-11-02.md)。各工程 TEST-REPORT 是历史记录，不能把旧版烧录结果当作本版本实测证明。

## 编译与测试

ESP32 开发使用 PlatformIO；工程配置固定为 `espressif32@7.1.3`、`esp32dev`、Arduino。安装 PlatformIO 后可在包根目录执行：

```powershell
pio run --project-dir motor-bringup
```

此命令只编译。需要安装工具链时会联网下载。包内没有 `.pio` 缓存。

若安装了 g++，可在包根目录运行全部 Python、8 组 C++ 测试及固件编译：

```powershell
.\run_checks.cmd --cpp --firmware
```

也可用自己的 Python 环境直接执行 `python release_tools/run_checks.py --cpp --firmware`。测试产物写到 `tmp/offline-checks`，不会访问真实串口或烧录设备。

`environment-snapshot.json` 记录打包电脑实际安装的版本，仅用于排查环境差异；安装仍以 `esp32-pc-link/pc/requirements.txt` 为准。源码包的逐文件 SHA-256 在 `source-manifest.json`；压缩包外的 `.zip.sha256.txt` 校验整个 ZIP，发布检查记录位于同名 `.检查.txt`。

## 实车调试边界

当前是每轮“前馈 + PI”，没有 D 项、IMU 或航向闭环。速度以 counts/s 与标定量程百分比表示，不能直接当成 mm/s。先按 [实机验收清单](motor-bringup/CLOSED-LOOP-VERIFICATION.md) 做架空标定与轮速跟随，再比较落地运动效果；标定会达到 100% 输出，必须四轮完全架空。

源码离线通过不代表实车已验收。发包过程不连接设备、不烧录、不转动电机；队友连接自己的开发板后需核对引脚、实际轮位和标定。

## 再次生成源码包

在项目根目录用 Python 执行：

```powershell
python release_tools/build_source.py --output-dir output/releases/new-source
```

脚本按白名单选择源码，自动排除本地环境、编译产物、历史包和个人配置。默认不会覆盖同名已存在的发布包。文件名中的版本是明确的源码快照标识，发布新版本时同步调整脚本中的版本与包名。
