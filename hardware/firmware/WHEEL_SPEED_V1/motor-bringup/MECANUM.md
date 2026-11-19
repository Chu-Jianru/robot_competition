# 本车麦轮控制接口

本车 V3 / FOUR_V2 的引脚、极性和轮位统一见 [固件说明](../../README.md)，现场流程见 [操作手册](../../../../docs/手机接入与自主清扫手册.md)。当前菜单布局为 CBDA，按左前、右前、左后、右后排列，四个布局符号均为 +1。

## 坐标与混控

车体坐标 x 向前、y 向左、z 逆时针。百分比混控为 FL=x-y-z、FR=x+y+z、RL=x+y-z、RR=x-y+z；超出幅度时四轮同比缩放，再映射到 A/B/C/D，底层 `motor_profile.h` 转换电气极性。

编码器原始计数按通道排列，反馈方向由标定符号修正。轮速单位为 counts/s，已含四倍频。车体物理速度到四轮计数的换算由 Windows 菜单 11 标定并保存。

## 接口用途

- `MECANUM`、`WHEELS` 为定时开环百分比动作。
- `SPEED`、`SPEEDW` 为定时轮速闭环动作，目标百分比按统一 Vmax 换算。
- `MECANUM_MANUAL` 为菜单 13 可选的持续 PWM 控制，租期 250 ms，配合 `MECANUM_KEEP` 续期。
- `SPEED_STREAM` 为连续四轮计数/秒目标，菜单 10/11 和菜单 13 闭环模式使用，租期 150 ms。
- `CAL`、`CAL?`、`PID`、`PID?`、`SAVE` 用于标定、参数查询与 NVS 保存。
- `STOP` 关闭输出；控制端 PING 心跳与各动作租期分别维护。

连接先核对能力、停止状态、LAYOUT 与标定。协议实际语法、回包和能力字段在 `src/main.cpp`，Python 链路在 `windows/chassis_link.py` 与本目录 `chassis_link.py`，调用参数按所在模块核对。

## 控制与故障

轮速反馈经窗口测速，再由目标斜坡、前馈与 PI 计算 PWM。持续流的速度上限和 PWM 限幅由固件单独约束。STOP、故障和租期耗尽时停车，机械滑行距离需现场验收。

`encoder_sign`、`no_feedback` 等故障需保存完整回包、轮号、目标/实测速和 PWM。读数与处理顺序见 [调试指南](DEBUGGING.md)，实车项目见 [闭环验收清单](CLOSED-LOOP-VERIFICATION.md)。

队友原始的定时动作协议和完整开发记录保留在 [vendor 原始包](../../../../vendor/README.md)。
