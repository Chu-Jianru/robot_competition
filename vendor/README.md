# 队友源码包（只读参考）

`PushRobot-Source-20261102-WHEEL_SPEED_V1/` 是**队友**于 2026-11-02 提供的源码包，不是本项目自研代码，整体只读，不改、不并入 `hardware/workbench/source`。

> `hardware/workbench/`（约 184 MB 封存工作台）只在本地运行，已在 `.gitignore` 排除；从仓库 clone 后没有这个目录，本文里它的路径只作历史对照。队友源码仍在 `vendor/`，固件工作副本在 `hardware/firmware/`。

- 版本 **WHEEL_SPEED_V1**：保留开环 MECANUM_V2、FOUR_V1 接线与 CBAD 轮位，新增每轮"前馈 + PI"的轮速闭环（`motor-bringup/src/wheel_speed.h`）与标定流程。
- 本包不带 EXE、不带 `.venv`/`.pio`、不带个人设置和标定；启动方式见包内 `README.md`（`setup_source.cmd` → `run_debug.cmd`）。
- 包内 `source-manifest.json` 是 93 个文件的 SHA-256 清单；移动时保持目录结构原样，否则校验和内部相对路径都会失效。
- 包内各 `TEST-REPORT.md` 是**历史记录**，不能当作本版本已实车验收；其明示"每轮前馈 + PI，无 D 项、无 IMU/航向闭环，counts/s 不等于 mm/s"。

用途：作为交接中"下一步第 1、2 项"（编码器/机械参数核对、MCU 周期测速与四轮速度闭环）的参考实现与验收清单（`motor-bringup/CLOSED-LOOP-VERIFICATION.md`）。是否采用、何时合并进现用固件，需另行单独决策。

## 与现有目录的对应

### 1. 子工程级

| 队友包内路径 | 本项目现有位置 | 与我方自研代码的关系 |
|---|---|---|
| `motor-bringup/`（固件 + PC 端串口类） | 旧版在 `hardware/workbench/source/motor-bringup/`，新版在本包 | 我方无对应实现；ROS 侧只有 `src/sweep_robot/sweep_robot/mecanum_ik.py` 模板，其 `twist_to_wheels()` 仍是 `NotImplementedError`，不能互替 |
| `debug-workbench/`（调试界面） | 旧版在 `hardware/workbench/source/debug-workbench/` | 我方没有对应物；`windows/` 是比赛视觉，不是底盘界面 |
| `esp32-pc-link/lib/link_core/`（DRY_RUN 协议核心） | 旧版在 `hardware/workbench/source/esp32-pc-link/lib/link_core/` | 被固件直接 include，删掉编译不过；我方无对应 |
| `esp32-pc-link/pc/`（串口客户端、面板、相机预览） | 旧版在 `hardware/workbench/source/esp32-pc-link/pc/` | `camera_preview.py` 与我方 `windows/usb_camera.py` 功能相近但**不是同一套**，不合并 |
| `release_tools/`（打包、自检、发布校验） | 旧版在 `hardware/workbench/source/release_tools/` | 我方无对应 |
| `app/`（免安装 EXE + 依赖） | 只在 `hardware/workbench/app/`（182 MB） | 打包产物，本包不带 |
| `docs/superpowers/` | 只在本包 | 闭环设计与实施计划，对应交接"下一步第 1、2 项" |
| `esp32-connection-test/` | 只在本包 | 最小 USB/PING 排障，不参与正常启动 |
| `source-manifest.json`、`environment-snapshot.json`、`.gitignore` | 只在本包 | 发布清单（93 条 SHA）、打包机环境快照 |

### 2. 两份包的逐文件比对

以 `hardware/workbench/source/`（76 文件）为基准，与本包同名文件逐字节比较：

| 类别 | 数量 | 代表文件 |
|---|---:|---|
| 逐字节相同 | 37 | `mecanum.h`、`link_core.*`、`esp32-pc-link/pc/panel*.py`、`debug-workbench/bench_app.py`、`camera_view.py` |
| 内容不同 | 25 | `motor-bringup/src/main.cpp`、`src/motor_profile.h`、`chassis_link.py`、`motor_link.py`、`MECANUM.md`、`FOUR-MOTOR-WIRING.md`、`debug-workbench/chassis_app.py`、`run_checks.cmd`、`setup_source.*` |
| 只在旧包 | 14 | 旧入口 `run_chassis.cmd`、`run_panel.*`、`test_panel.py`；旧打包工具 `build_share.py`、`verify_share.py`、`desktop_entry.py`、`release_info.py`、`使用说明.md` |
| 只在本包 | 32 | 闭环与蓝牙源码 `src/wheel_speed.h`、`src/calibration.h`、`src/config_store.h`、`src/wireless.h`；5 个新 C++ 测试；`BLUETOOTH.md`、`BLUETOOTH-TEST-REPORT.md`、`CLOSED-LOOP-VERIFICATION.md`、`DEBUGGING.md`；`docs/superpowers/`；`build_source.py` |

### 3. 会改变实车行为的差异（未合并，禁止混用）

| 项 | 旧包（现用固件 MECANUM_V2） | 本包（WHEEL_SPEED_V1） | 影响 |
|---|---|---|---|
| 轮位 | ADBC：FL/FR/RL/RR = A/D/B/C，面板全 +1 | CBAD：左前 C、右前 B、左后 A、右后 D，面板全 +1 | 只烧新固件而不改面板轮位，前进会变斜行或反向 |
| 电气极性 `motor_profile.h` | `{-1,+1,+1,-1}` | `{-1,-1,-1,-1}`（2026-10-27 逐路复核：B/C 正输出朝车尾） | 该修正要烧录才生效，只看源码会误以为车已改 |
| 协议 | `MECANUM`/`WHEELS`/`JOG`，百分比 + 定时 | 增 `SPEED`/`SPEEDW`/`CAL`/`PID`/`TELEM`/`SAVE` 与 `CHASSIS_CAPS2` 能力行 | 旧面板连新固件仍走开环；新面板连旧固件闭环控件自锁 |
| 通信 | 仅 USB 串口 | 增 ESP32 经典蓝牙 SPP（`PushRobot-ESP32`，PIN 271038），USB/蓝牙互斥 | 与比赛要求的 Wi-Fi 链路无关，不要混为一谈 |
| 闭环前提 | 无 | 必须四轮悬空完成标定，未标定拒绝闭环命令 | 标定会到 100% 输出，必须完全架空 |

### 4. 三处"看起来像、其实不是"

- `esp32-pc-link/pc/camera_preview.py`（队友的调试图传） ≠ `windows/usb_camera.py`（我方比赛取图 + 红块检测）。
- 队友的百分比麦轮混控 ≠ ROS 的 `mecanum_ik.py`：前者是开环百分比与定时动作，后者是 m/s、rad/s 逆解模板且尚未实现。
- 队友 `TEST-REPORT.md` 里的"通过" ≠ 本车验收：那是他们那台车的记录，本车的编码器计数、极性与标定仍未核对。
