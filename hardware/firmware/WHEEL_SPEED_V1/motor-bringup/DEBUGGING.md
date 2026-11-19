# 底盘源码与调试指南

本车接口与参数入口见 [固件说明](../../README.md)，现场报错处理见 [操作手册](../../../../docs/手机接入与自主清扫手册.md)。

## 阅读顺序

- `src/main.cpp` 处理 GPIO、编码器、命令分发、控制权、标定、速度流和故障回报。
- `src/wheel_speed.h` 实现窗口测速、前馈/PI、积分限幅与故障判定。
- `src/calibration.h` 计算两点拟合与速度量程。
- `src/config_store.h` 处理标定和增益的 NVS 保存与读取。
- `src/mecanum.h` 完成 X 型混控和轮位映射。
- `src/motor_profile.h` 是逻辑方向到电气极性的转换入口。
- `src/wireless.h` 管理蓝牙发送、连接代次与回复路由。
- 本目录 `motor_link.py`、`chassis_link.py` 为工作台串口适配；Windows 任务链路为 `windows/chassis_link.py`。

## 顺序与单位

四轮固件数组、编码器与遥测按 A/B/C/D 排列，混控先按左前/右前/左后/右后计算再映射。本车当前菜单布局为 CBDA，电气极性与编码器反馈符号分别处理。

速度以 counts/s 表示，原始计数允许正向运行时递减，需使用标定符号归一化。PID 时间使用秒，固件高分辨率计时使用微秒，指令时长和租期使用毫秒。Windows 与 ESP32 时钟各自计时。

`LIMIT` 影响统一 Vmax，流式目标上限和 PWM 限幅另由固件约束。按源码区分目标轮速、斜坡轮速、测得轮速和实际占空比。

## 排查顺序

1. 保存 WHO、CAPS、POLARITY、LAYOUT、CAL?、SPEED_INFO 的回包，核对本车配置与最近烧录记录。
2. 遇方向故障，逐路核对机械轮位、电气方向、编码器原始增量和标定 sign。
3. 遇 `no_feedback`，比较故障轮的目标/实测轮速与 PWM，检查机械卡阻、供电和编码器。
4. 遇 `stream_timeout` / `link_timeout`，检查相机帧龄、速度目标有效期、心跳、串口回应和蓝牙连接。
5. 同步保存原图、标定、完整日志和固件版本，按 [闭环验收清单](CLOSED-LOOP-VERIFICATION.md) 复测。

## 离线检查

在项目根目录运行本车客户端假串口测试。

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' -B -m unittest discover -s hardware/firmware/WHEEL_SPEED_V1/motor-bringup/tests -p 'test_*.py'
```

`tests/` 同时保存轮速、标定、NVS、控制入口、电机与蓝牙的 C++ 主机测试。固件编译方式见 [工具说明](../../../../tools/README.md)。队友原始工作台说明和测试脚本在 [vendor 原始包](../../../../vendor/README.md)。
