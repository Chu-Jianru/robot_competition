# 本车固件 V3

本车工作副本基于队友 2026-11-02 的 WHEEL_SPEED_V1，开发目录为 [motor-bringup/](WHEEL_SPEED_V1/motor-bringup/README.md)，相对依赖为 `WHEEL_SPEED_V1/esp32-pc-link/lib/link_core/`。

`vendor/PushRobot-Source-20261102-WHEEL_SPEED_V1/` 保存原始源码，`archive/deliverables/` 保存原始 zip。`tools/核对固件版本.cmd` 比较工作副本与归档包。`hardware/workbench/` 为本地封存旧版。

## 本车引脚、极性与轮位

当前源码与实车回读采用 FOUR_V2。每路依次为 IN1、IN2、编码器 A、编码器 B。

- A 路为 `22, 23, 21, 19`。
- B 路为 `17, 18, 16, 4`。
- C 路为 `26, 25, 27, 14`。
- D 路为 `32, 33, 35, 34`。
- STBY 为 GPIO13，GPIO34/35 为输入。

`src/motor_profile.h` 的 A/B/C/D 电气极性为 `{1, -1, -1, 1}`。轮位在连接时发送 `LAYOUT` 设置，当前菜单 9/13 默认 `CBDA 1 1 1 1`，菜单 10 读取本机 `windows/task_drive.json`。`ChassisLink` 的通用默认仍为 ADBC，直接调用脚本时需核对本车布局。

## 协议与运行

- 菜单 10/11 使用 `CHASSIS_SPEED_STREAM`，四轮目标以编码器计数/秒表示，持续闭环。车体位姿到轮速的换算由菜单 11 保存。
- 菜单 13 默认使用 `SPEED_STREAM`，也可选择 `MECANUM_MANUAL` 直接 PWM。
- 速度流租期为 150 ms，直接 PWM 租期为 250 ms；超时停车。链路还维护心跳与独立失联处理。
- `CAL` 的四轮标定存入 NVS，查询 `CAL?`、`SPEED_INFO` 可核对。编码器方向和目标/实测速差故障按固件保护处理。

编译与烧录统一见 [tools/README.md](../../tools/README.md)，现场连接、标定和故障处理见 [操作手册](../../docs/手机接入与自主清扫手册.md)。最新烧录及静止回读记录见 [交接说明](../../交接说明.md)。
