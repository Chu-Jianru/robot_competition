# 固件工作副本：WHEEL_SPEED_V1

用于把队友 2026-11-02 的固件烧到本车 ESP32 的**可写构建目录**。`vendor/` 里的原件保持只读、带 93 条 SHA 清单，不在那里编译或修改。

## 同名内容一共三处，各管一件事

| 位置 | 角色 | 能不能改 |
|---|---|---|
| `vendor/PushRobot-Source-20261102-WHEEL_SPEED_V1/` | 队友原始解压包，对照基准 | **不能**（只读，SHA 清单） |
| `hardware/firmware/WHEEL_SPEED_V1/` | 本车的构建副本，改极性、后续改流式接口都在这 | 可以 |
| `archive/deliverables/PushRobot-Source-20261102-WHEEL_SPEED_V1.zip` | 收到的原始压缩包，归档留证 | 不动 |

三者内容是同一版：zip 与 `vendor/` 94 个文件逐字节相同，工作副本除 `motor_profile.h` 一行外与之相同。改动后想确认有没有漂移，双击 `tools\核对固件版本.cmd`（或跑 `tools/check-firmware-sync.ps1`）。

**旧版固件只有一处**：`hardware/workbench/source/motor-bringup/`（MECANUM_V2，无蓝牙），只用于回退，不是当前开发目标。

## 来源与范围

- 来源：`vendor/PushRobot-Source-20261102-WHEEL_SPEED_V1/`（2026-11-02）。
- 只复制了固件真正需要的两棵子树：
  - `motor-bringup/`（`main.cpp` + 各头文件 + 离线测试）；
  - `esp32-pc-link/lib/link_core/`（`main.cpp` 与 `dry_core.cpp` 用相对路径引用它）。
- 未复制 `debug-workbench/`、`release_tools/`、`docs/` —— 那是队友的上位机和打包工具，不是固件依赖。

## 与原件唯一的差异

`motor-bringup/src/motor_profile.h` 的四路电气极性：

```cpp
// 原包（队友车）：{-1, -1, -1, -1}
constexpr int kForwardPolarity[4] = {-1, 1, 1, -1};   // 本车，取自 hardware/workbench 已实跑值
```

不改这一行，B/C 两轮会反着转。轮位**不写在固件里**，是运行期命令 `LAYOUT ADBC 1 1 1 1`；我们的 `windows/chassis_link.py` 连接后会自动发。

## 编译与烧录

PlatformIO，`platformio.ini` 固定 `espressif32@7.1.3` / `esp32dev` / 115200。

**VS Code**：打开 `hardware/firmware/WHEEL_SPEED_V1/motor-bringup` → 等平台加载完 → 选 COM 口 → Upload。

**CLI**（装在项目内，不写系统目录）：

```powershell
python -m venv tools\pio-venv
tools\pio-venv\Scripts\python.exe -m pip install -U platformio
$env:PLATFORMIO_CORE_DIR = "$PWD\.pio-core"
tools\pio-venv\Scripts\pio.exe run -d hardware\firmware\WHEEL_SPEED_V1\motor-bringup -t upload --upload-port COM4
```

编译产生的 `.pio/`、`.pio-core/` 已在 `.gitignore` 里。

## 烧后验证（先只读，不动电机）

```
WHO          -> UNIFIED_ROBOT 1 mode=DRY_RUN
CAPS         -> 应含 CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1
             -> 应含 LINK_CAPS USB=1 BT=SPP OWNER=EXCLUSIVE NAME=PushRobot-ESP32
POLARITY     -> MOTOR_FORWARD A=-1 B=1 C=1 D=-1   （与副本一致）
STATUS       -> STOPPED ALL
```

然后架空车轮、断驱动电源以外的前提都确认后：

```
MODE CHASSIS
LAYOUT ADBC 1 1 1 1
MECANUM 15 0 0 300
```

四轮都应朝车头。开环 `MECANUM` **不需要标定**；`SPEED` / `SPEEDW` 闭环需要先 `CAL`，标定结果存 NVS（`SAVE`），不用重烧。

## 尚未做

- **已编译、已烧录**（2026-11-02 23:00 前后，COM3，4 MB Flash，四个分段 `Hash verified`）；烧后 `WHO`/`CAPS`/`POLARITY` 通过，得到 `WHEEL_SPEED_V1`、`BT=SPP`、`A=-1 B=1 C=1 D=-1`。
- 固件镜像 SHA256：`EB4978344A49CDBD56BBFC43CF4857188047CD58E0FF3410E82565CE77BF802C`（1140624 字节），重新编译可复现同一哈希。
- 轮速闭环的每圈计数、符号、死区、轮径轮距标定未做（`CAL=0`，`SPEED` 暂不可用）。
- 2026-11-03 蓝牙静止连接已验证：`PushRobot-ESP32` / `2CBCBB9F962E`，本次 Windows 分配 COM6 传出、COM7 传入；新版面板改选 COM6 后握手成功、持续收到状态与四路编码器，未发送运动指令、未应用轮位。此前将 `…962E` 判为队友另一块板的依据不足：本地 SDK 为四地址配置，蓝牙 MAC 默认是基础 MAC +2，与烧录记录 `…962C` 的派生关系一致。端口重新配对后可能变化，以实际传出端口为准。当前设备还报告 `CAL VALID=1`、`MOTOR_DIR_V1`、`HOLD_V1`，尚未核验当前镜像和标定来源，不将 11-02 的镜像/未标定状态当作当前实测。
- `LAYOUT` 不持久化：每次上电或切换模式后都要重发。
- 蓝牙名 `PushRobot-ESP32` / PIN `271038` 是编译期常量，要改名必须改 `wireless.h` 再烧。

## 原始压缩包

`PushRobot-Source-20261102-WHEEL_SPEED_V1.zip` 已归档到 `archive/deliverables/`，不放在本目录，避免和构建副本混在一起。它与 `vendor/` 中解压的 94 个文件逐文件 SHA256 一致，也是本副本的来源；`tools/核对固件版本.cmd` 会从归档位置取它做比对。


## 回退

烧 `hardware/workbench/source/motor-bringup`（MECANUM_V2，无蓝牙），回到 2026-10-22 已验证的状态。
