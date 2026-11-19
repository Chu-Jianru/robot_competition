# 本车 ESP32 固件工作副本

本目录为 V3 / FOUR_V2 固件，来源为队友 WHEEL_SPEED_V1。当前引脚、极性、CBDA 轮位及速度流说明统一见 [本车固件说明](../../README.md)，编译与烧录见 [工具说明](../../../../tools/README.md)。

## 源码与检查

- `src/main.cpp` 处理命令、控制权、流式目标、心跳与故障回报。
- `src/motor_profile.h` 定义本车电气极性。
- `src/dry_core.cpp` 与主程序相对引用 `../esp32-pc-link/lib/link_core/`。
- `platformio.ini` 固定编译环境，`tests/` 保存主机侧固件检查。
- [源码与调试指南](DEBUGGING.md) 说明轮速控制与参数。
- [闭环验收清单](CLOSED-LOOP-VERIFICATION.md) 说明本车现场验收记录项目。
- [接线说明](FOUR-MOTOR-WIRING.md)、[蓝牙连接](BLUETOOTH.md) 提供本车配置。
- [麦轮协议](MECANUM.md) 保留定时动作的坐标和混控接口。

旧版开发与测试报告保留于 [vendor 原始包](../../../../vendor/README.md)。本车最新实机状态见 [交接说明](../../../../交接说明.md)。
