# 蓝牙功能验证记录

日期：2026-10-27。目标：ESP32-WROOM-32 内置蓝牙连接 Windows 现有麦克纳姆底盘面板。

## 结果

- Espressif32 7.1.3 / Arduino-ESP32 2.0.17 编译通过，沿用原默认分区。Flash 1,119,597 / 1,310,720 字节；静态 RAM 40,996 / 327,680 字节。
- COM4 烧录成功，镜像哈希验证通过；芯片识别为 ESP32-D0WDQ6。未擦除整片 Flash，原引脚和通道方向不变。
- Windows 已配对 `PushRobot-ESP32`，蓝牙地址 `2C:BC:BB:9F:96:2E`。本机 COM5 对应远端串口服务，COM6 是传入端口。
- 26 项串口客户端测试、38 项面板/数据/路径测试通过。
- 三组 C++ 固件模拟测试 `test_motor`、`test_chassis`、`test_bluetooth` 通过；覆盖旧电机/麦轮行为及新增双端隔离、控制权、停止、失联、重连、队列故障和时间回绕。
- USB 静止查询确认 `BT=SPP`、MECANUM_V2、A/B/C/D 极性全 -1；STBY=0，八路 PWM 均为零。
- 通过实际蓝牙 COM5 运行现有底盘面板：握手就绪、收到四路编码器、连接后布局未确认、运动按钮禁用。台架/模拟模式切换均保持未连接，无自动动作。
- 实际蓝牙断开重连后返回 DRY_RUN；重新进入底盘模式可确认 `LAYOUT CONFIGURED=0`。配置布局后故意不发心跳，约 1.5 秒后收到 `STOPPED link_timeout` 和会话重置标记；重新进入底盘模式仍需重新确认布局。
- 验证结束为 DRY_RUN，STBY 和全部 PWM 为零，COM4/COM5 测试连接均已关闭，可供用户面板使用。

## 实机范围

实机只发送停止、查询、模式切换和不产生运动的布局配置。**没有发送 JOG、WHEELS 或 MECANUM 运动指令。** 编码器静止计数为零不证明电机/编码器运动正常。带载无线距离、电源稳定性、真实转动及失联后的机械停止过程仍需现场测试。

首次界面测试受受限进程访问 Tcl/Tk 运行库影响，重试正常权限的测试进程后 38 项全部通过。初次自动配对未完成，用户随后在 Windows 中完成配对，再通过 COM5 完成静止验收。

## 原始记录

- [USB 蓝牙能力和零输出查询](bluetooth-usb-static.txt)
- [现有面板通过 COM5 的静止验收](../debug-workbench/records/chassis-static-20261027-160138.json)
- [COM5 重连与心跳超时的静止验收](bluetooth-reconnect-static.json)
