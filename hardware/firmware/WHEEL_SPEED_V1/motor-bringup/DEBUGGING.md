# 底盘源码与调试指南

本指南对应 `WHEEL_SPEED_V1`。源码整理日期：2026-11-02。整理保留现有控制参数、引脚、协议与运行行为；没有烧录或试转小车。

## 1. 从哪里开始读

| 文件 | 职责 | 调试入口 |
|---|---|---|
| `src/main.cpp` | GPIO、编码器采集、自动标定、控制调度、命令分发 | 按源码的 1～7 节导航阅读 |
| `src/wheel_speed.h` | 测速窗口、前馈 + PI、故障判据，纯计算 | `SpeedEstimator`、`WheelPid::step`、`FaultMonitor::update` |
| `src/calibration.h` | 两点拟合与统一速度量程计算 | `computeLine`、`computeVmax` |
| `src/config_store.h` | ESP32 NVS 掉电保存与读回校验 | `load`、`saveCalibration`、`saveGains` |
| `src/mecanum.h` | X 型麦轮混控、轮位到通道映射 | `mix` |
| `src/motor_profile.h` | 逻辑正向到电气极性的唯一转换 | `kForwardPolarity` |
| `src/wireless.h` | USB/蓝牙回复路由与蓝牙发送队列 | `controlOwner`、`replyPort`、连接 `epoch` |
| `motor_link.py` | 后台串口线程、握手、心跳、优先 STOP | `_run`、`_enqueue` |
| `chassis_link.py` | 底盘协议封装、闭环应答转事件 | `speed`、`handle_response` |
| `../debug-workbench/chassis_app.py` | 界面操作、事件处理与按钮状态 | `run_motion`、`pump`、`update_controls` |

## 2. 三条主要调用链

```text
用户点击 → ChassisApp.run_motion → ChassisLink.motion / speed → 发送队列
  → MotorLink._run → USB/蓝牙 → receiveByte → dispatchCommand
  → handleCommand → handleChassis → 开环输出 / startClosedLoop

闭环：serviceControl → 原始计数快照 → 反馈方向归一化 → 滑动窗口测速
  → 目标斜坡 → 每轮前馈 + PI → setMotorForwardPercent → PWM
  → 每轮故障监视 → 任一路异常则四轮停车并闭锁

回报：固件回复 → MotorLink._run → ChassisLink.handle_response → events 队列
  → ChassisApp.pump → 更新日志、表格和按钮
```

Python 方法返回 `True` 只表示请求已入队。看到 `SPEED_RUN` / `CHASSIS_RUN` 才表示固件接受运行；这仍不能代替观察实际轮子转动。串口线程不能直接更新 Tk 控件。

## 3. 先分清顺序、符号与单位

- 固件四轮数组、`SPEEDW`、`WHEELS`、`TLM` 均按 **A/B/C/D** 排列。
- 麦轮公式先按 **左前/右前/左后/右后** 计算，再按 `Layout` 写入各通道。当前实测布局为 **CBAD**：左前 C、右前 B、左后 A、右后 D。
- 逻辑正输出表示朝车头滚动；输出极性由 `motor_profile.h` 修正，界面方向保持全 +1。
- `calibrationData.wheel[i].sign` 是编码器反馈方向修正，只作用于测速；原始 `ENC ticks` 允许正向运行时递减。
- 编码器速度单位为 **counts/s**，已含四倍频，不要再乘 4。没有实测每圈计数时不能直接称为 RPM。
- `MECANUM/WHEELS` 的百分比是开环输出；`SPEED/SPEEDW` 的百分比是统一速度量程 `Vmax` 的比例。
- `LIMIT` 用于计算 `Vmax = LIMIT / 100 × 四轮同时满输出测速的最小值`，**不是 PWM 输出限幅**；PI 输出仍可能达到 ±100%。
- `vsim` 是“四轮同时运行”的实测速率，不是仿真数据。`DEADBAND=a` 是两点拟合的截距，不是独立测得的最低起转占空比。
- `nowUs/dtUs/lastControlUs` 使用微秒；协议时长与其余调度时间使用毫秒；PI 的 `dt` 使用秒。不要混用电脑 `monotonic()` 与 ESP32 的时间戳。

## 4. 看哪些变量和回报

| 变量 / 回报 | 含义与排查方向 |
|---|---|
| `goalTarget[i]` | 用户指令混控后的最终目标，counts/s |
| `rampedTarget[i]` | 斜坡后的目标，TLM 中显示此值；起步阶段低于最终目标正常 |
| `measuredSpeed[i]` | 窗口平滑、符号归一化后的实测轮速 |
| `appliedDuty[i]` | 控制器计算的逻辑输出百分比，写 PWM 前取整 |
| `calibrationData.valid/vmax` | 闭环是否可用、统一速度量程；失败标定会作废旧标定 |
| `closedLoopActive` / `running` | 闭环激活 / 定时动作运行；标定使用独立的 `calibrating` |
| `faultLatched/lastFaultReason` | 故障闭锁及原因，单纯静止不代表已清除 |
| `ENC ... invalid=...` | 非法编码跳变累计数；增长需查信号、接线和中断负载 |
| `CAL PROGRESS` / `CAL FAIL` | 标定阶段 / 失败原因；轮号 0..3 对应 A..D |

遥测格式固定为：

```text
TLM <ESP32毫秒> <目标A> <目标B> <目标C> <目标D> <实测A> <实测B> <实测C> <实测D> <输出A> <输出B> <输出C> <输出D>
```

共 13 个数值，目标/实测为 counts/s，输出为 %，均取整。目标是斜坡后的值。停止后控制状态被清零，TLM 不表示停车过程中的真实滑行速度。USB 缓冲空间不足时允许跳过一次遥测，标定期间不发 TLM。

需要观察时使用“闭环调参”页；高频 TLM 只更新表格，不刷日志。不要在编码器中断或 10 ms 控制路径中添加逐次串口打印，也不要用断点暂停正在驱动电机的固件；时序测试优先使用离线模拟或架空、受约束的测试环境。

## 5. 常见现象按什么顺序查

| 现象 | 优先检查 |
|---|---|
| 连接成功但按钮未开放 | 原始日志的 `WHO → MODE_OK → PONG → CAPS → STOPPED`；真实引脚版本应为 FOUR_V1，闭环另需 CAPS2；再核对布局和现场确认 |
| 提示需要标定 | `CAL?` 的 VALID；查看 `CAL FAIL`，不要靠硬改 valid 绕过检查 |
| `CAL FAIL no_feedback i` | 对应轮编码计数、供电与接线；流程先试 60%，无足够反馈时重试一次 75% |
| `CAL FAIL bad_line i` | 两点速度是否递增、满输出速度是否足够、拟合截距是否合理 |
| `CAL FAIL bad_sim i` | 四轮同时输出时该轮是否仍有足够反馈，检查供电压降与负载 |
| `CAL FAIL nvs` / 重启丢标定 | NVS 打开/写入结果、结构版本与合法性校验；标定自动保存，修改增益后要单独 SAVE |
| `encoder_sign` | 标定后反馈方向是否与输出一致、是否换过编码线/电机/轮位；先查方向，不能靠调大增益解决 |
| `no_feedback` / `stall` | 计数是否变化、车轮是否卡住、驱动是否供电、编码器是否断线；仅靠轮速不能严格区分电气与机械原因 |
| 低速命令被拒绝 | 混控后每个非零轮目标必须至少为 5%；不是只检查输入 x/y/z |
| 目标/实际持续来回摆动 | 先看测速波动、供电与机械阻力，再逐步调 P/I；每次改一个参数并保留对照记录 |
| 四轮轮速稳定但车仍走偏 | 查打滑、四轮接地、滚轮安装、轮径差与布局；本版没有车体航向/视觉闭环 |
| `link_timeout` / 蓝牙异常 | STATUS 心跳、控制端归属、另一面板占用、发送队列失败；持续收到 TLM 不代表固件收到心跳 |

故障触发后先排查原因，再显式 `STOP`。全零 `SPEED` 只停车，**不会**清除故障闩锁；`STOPPED ALL` 也只是静止状态。切换模式或撤销控制权同样会清锁，因此验证闭锁时不能中途断开面板。

## 6. 参数入口与修改边界

- `main.cpp` 顶部集中列出控制周期、最大控制间隔、遥测周期、PWM 与标定测量时长。控制正常为约 10 ms；间隔超过 50 ms 时清 PWM、测速和积分，再等下次正常服务。
- `wheel_speed.h::Gains`：默认 Kp=2、Ki=1、LIMIT=80、AMAX=300。已有 NVS 增益会覆盖源码默认值，改默认值不代表已连接小车立即采用新值。
- 界面 Kp/Ki 使用小数；线上 `PID` 使用千分整数，例如 2500 对应 2.500。当前没有 D 项。
- `SpeedEstimator::kWindow`：4 个样本，正常节拍下约 40 ms；加大窗口会增加反馈延迟。
- `FaultMonitor` 私有常量：方向反约持续 0.3 s、无反馈约 0.8 s、堵转约 1 s 才判故障，且分别还有输出与速度门槛。排查时必须同时看门槛和持续时间。
- 启动/停止会清积分；积分修正项和最终输出均有限幅。不要通过放宽故障阈值掩盖反馈异常。

## 7. 如何验证修改

已有离线测试覆盖：

| 测试文件 | 重点 |
|---|---|
| `tests/test_wheel_speed.cpp` | 测速、PI、积分饱和、低速模型与故障 |
| `tests/test_calibration.cpp` | 前馈拟合与统一量程 |
| `tests/test_config_store.cpp` | 保存/回读、无效数据、写失败 |
| `tests/test_closed_loop.cpp` | 真实固件入口的闭环命令、标定、遥测、故障 |
| `tests/test_motor.cpp`、`test_chassis.cpp`、`test_bluetooth.cpp` | 原台架、麦轮混控、双端控制与断链 |
| `tests/test_*.py` | 台架/底盘客户端与假串口往返 |
| `../debug-workbench/tests/test_*.py` | 工作台、底盘界面、状态门控 |

在项目根目录可运行 Python 测试（GUI 测试需可用桌面）：

```powershell
& .\esp32-pc-link\pc\.venv\Scripts\python.exe -m unittest discover -s motor-bringup/tests -p 'test_*.py'
& .\esp32-pc-link\pc\.venv\Scripts\python.exe -m unittest discover -s debug-workbench/tests -p 'test_*.py'
```

离线编译固件（不上传）：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run --project-dir motor-bringup
```

真实测试从根目录 `run_debug.cmd` 启动，在顶部选择麦克纳姆底盘。自动标定会以最高 100% 输出运行车轮，必须先四轮完全架空。按 [实机验收清单](CLOSED-LOOP-VERIFICATION.md) 完成悬空标定、重启回读、轮速跟随和落地对照。测试通信超时必须保持 ESP32 供电，拔掉供电 USB 不等于测试固件失联停车。离线通过不能替代实际运动效果验收。
