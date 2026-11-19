# 硬件与底盘

本车使用 ESP32、D24A 驱动板和四轮麦克纳姆底盘，通过经典蓝牙 SPP 接收 Windows 指令。

- [本车固件](firmware/README.md) 说明 V3 / FOUR_V2 引脚、极性、轮位和源码来源。
- [本车接线说明](firmware/WHEEL_SPEED_V1/motor-bringup/FOUR-MOTOR-WIRING.md) 以当前源码为准，[接线表](ESP32与D24A当前接线表.xlsx) 保留原接线资料，历史版本须核对 FOUR_V2 引脚。
- [操作手册](../docs/手机接入与自主清扫手册.md) 说明轮速标定、车体标定、遥控和自主清扫。
- [工具说明](../tools/README.md) 提供面板、PlatformIO 编译与烧录命令。

当前开发目录为 `firmware/WHEEL_SPEED_V1/motor-bringup/`，其相对依赖为同级 `esp32-pc-link/lib/link_core/`。新版面板用 [启动新面板.cmd](../tools/启动新面板.cmd)，菜单 9 使用简版轮速标定窗口。

`workbench/` 为 2026-10-22 封存的完整交付包，仅保留在本机，包含旧独立程序及运行依赖。队友 2026-11-02 原始包在 [vendor](../vendor/README.md)，原始 zip 在 `archive/deliverables/`。当前状态见 [交接说明](../交接说明.md)。
