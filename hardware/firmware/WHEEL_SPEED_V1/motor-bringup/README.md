# D24A 统一固件：整车模拟与四路电机台架

源码阅读、变量单位、参数入口与故障排查见 [源码与调试指南](DEBUGGING.md)；轮速闭环实车测试见 [实机验收清单](CLOSED-LOOP-VERIFICATION.md)。

2026-11-01：麦克纳姆底盘新增轮速闭环（WHEEL_SPEED_V1）：`SPEED`/`SPEEDW`、`CAL`、`PID`、`TELEM`、`SAVE`。闭环需先在四轮悬空状态下完成标定；未标定时闭环命令被拒绝，开环 `MECANUM`/`WHEELS` 不受影响。详见 [MECANUM.md](MECANUM.md) 的“轮速闭环”章节，实机验收见 [CLOSED-LOOP-VERIFICATION.md](CLOSED-LOOP-VERIFICATION.md)。

2026-10-27：新增 ESP32 内置蓝牙串口，Windows 可继续用原底盘面板；配对名称 `PushRobot-ESP32`，PIN `271038`。USB/蓝牙控制权互斥，断线/心跳超时撤销运动和布局。连接及供电说明见 [BLUETOOTH.md](BLUETOOTH.md)。

2026-10-27 最新逐路复测修正 B/C：只翻转 B/C，A/D 保持，现四个通道电气极性均为 -1。轮位 CBAD、面板全 +1、GPIO 引脚不变；本次方向修正需烧录。

当前为 MECANUM_V2：单路 JOG、四路 WHEELS、麦克纳姆控制均由底层驱动统一为正数向车头。A/D 的电气反向固化在 `src/motor_profile.h`；2026-10-27 更新面板轮位为 CBAD（左前 C、右前 B、左后 A、右后 D），四个方向全部 +1。用户确认方向未变，本次只需重新应用轮位，不改固件或 FOUR_V1 引脚。

2026-10-22：增加 `MODE CHASSIS`，支持四路同时输出和 X 型麦克纳姆开环混控。原台架及模拟模式继续保留，电气引脚仍为 FOUR_V1。请先阅读 `MECANUM.md`，从根目录 `run_debug.cmd` 启动，在顶部选择麦克纳姆底盘；不要用旧分享包 EXE 寻找新功能。

双击项目根目录 `run_debug.cmd`，在窗口顶部选择电机台架；底盘调试在同一窗口顶部选择麦克纳姆底盘。同一固件保留 DRY_RUN，并支持 A/B/C/D 四路逐路测试、四路编码器采集；台架模式每次只运行所选一路。麦克纳姆底盘已支持四轮协同开环运动与逐轮轮速闭环（需先完成四轮悬空标定）；视觉闭环、物理单位运动学、IMU 航向和里程计仍未实现。

## 最终接线 FOUR_V1

先断电接线。表格按用户确认的 IN2、IN1、编码器 A、编码器 B 排列。

| 通道 | IN2 | IN1 | 编码器 A | 编码器 B |
|---|---|---|---|---|
| A | AIN2 → GPIO19 | AIN1 → GPIO21 | E1A → GPIO22 | E1B → GPIO23 |
| B | BIN2 → GPIO4 | BIN1 → GPIO16 | E2A → GPIO17 | E2B → GPIO18 |
| C | CIN2 → GPIO14 | CIN1 → GPIO27 | E3A → GPIO26 | E3B → GPIO25 |
| D | DIN2 → GPIO33 | DIN1 → GPIO32 | E4A → GPIO35 | E4B → GPIO34 |

- STBY → GPIO13，四路共用，由程序控制，不固定接高电平。
- PWMA/B/C/D → D24A 自身 5V 输出；调速由各路 IN1/IN2 的 PWM 实现。
- ESP32 与 D24A 共地。ESP32 由 USB 供电，D24A 沿用已确认的 6V 电池输入；驱动板 5V/3V3 输出不接 ESP32 电源脚。
- 编码器由 D24A 电机口的 3V3/GND 供电，反馈信号不得为 5V。GPIO34/35 仅输入，不启用内部上拉。四个同款、同线序电机分别插入对应电机口。
- 当前沿用用户选择的 ESP32 3.3V 直接控制试验；不代表已验证满足 TB6612 在 5V 逻辑供电时的高电平规格。

详细记录见 `FOUR-MOTOR-WIRING.md`。旧版 A 路 GPIO25/26 控制接线已作废，不能将旧固件用于当前接线。

## 使用面板

1. 退出 VS Code Monitor 和其他占用 COM4 的串口窗口。
2. 启动工作台，选择真实电机台架，连接 CP2102 对应端口。新面板会确认 `CAPS MOTORS=ABCD PINS=FOUR_V1`，旧引脚固件不会开放操作。
3. 选择 A/B/C/D。四路持续采集，图表、计数和 RPM 显示所选通道；每路独立标定和保存每圈计数。旧单路标定只迁移到 A。
4. 电机固定、输出轴悬空且供电正常后勾选现场确认，填写输出 1～100%、方向、持续时间，再开始本次测试。支持秒/毫秒，范围 1～2147483647ms。
5. 切换通道先请求全部 STOP、撤销现场确认，不自动运行新通道。停止按钮、Esc、断开和关闭也请求停止。
6. 测试记录和采样 CSV 带通道号。收到实际 JOG 确认后才建记录，末次计数必须来自同一路；缺少读数时明确标注。

counts/s 与 RPM 根据电脑接收间隔估算；转速换算需要每路实测每圈计数（已含四倍频）。未标定不猜测 RPM，运行计数差不等于硬件精确锁存计数。此前约 50% 开始起转仅为用户当时单电机现场观察。

## 轮速闭环（WHEEL_SPEED_V1）

2026-11-01 起，`MODE CHASSIS` 增加逐轮编码器速度闭环；台架和 DRY_RUN 不受影响。闭环不是默认行为：必须先完成四轮悬空标定，否则 `SPEED`/`SPEEDW` 返回 `ERROR calibration required`。

- `SPEED x y z ms`：闭环麦轮混控；`SPEEDW A B C D ms`：闭环逐轮直控。每路非零目标 ≥ 5% 标定满量程，全零等同 STOP。
- `CAL`：四轮悬空标定，约 12–15 s；逐轮 60%（不起转重试 75%）与 100%，再四轮同时 100% 测 `vsim`，成功后自动写 NVS 并回 `CAL DONE VALID=1 VMAX=...`。失败或中止保持 `valid=0`，闭环禁用。
- `CAL?`：回读 `VALID`、`VMAX`、`LIMIT` 和四轮 `SIGN/V100/DEADBAND/SLOPE`；故障闭锁时仍可读。`CAL RESET`：只清有效性，需重新标定。
- `PID <kp_milli> <ki_milli> <limit>` / `PID?`：默认 `Kp=2.0`、`Ki=1.0`、`LIMIT=80`；`limit` 决定 `Vmax = LIMIT% × min(vsim)`（标定有效时改 limit 会立即重算），仅停止时允许修改。
- `TELEM 0|1`：20 Hz `TLM`，13 个数值字段（时间、四轮目标/实际 counts/s、四轮占空比 %）；标定期间不发。STOP 后仍可能发送，但内容是 `resetClosedLoop` 清零后的目标/实际/占空比（占空比为 0），不是停车瞬态。USB 缓冲不足时整行跳过，不阻塞控制 tick。`SAVE`：把标定与 PID 参数写入 NVS。
- `CAPS` 新增 `CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI`；旧面板只认旧能力行，继续走开环。
- 故障 `encoder_sign`（300 ms）、`no_feedback`（800 ms，绝对判据 `|u|≥85 && |v|<0.02×Vmax` **或**目标相对判据 `|target|>0 && |u|≥50 && |v|<0.20×|target|`）、`stall`（1000 ms）会停车并置闩锁；`SPEED`/`SPEEDW`/`CAL` 被拒绝，必须显式 `STOP`（或切换模式/断链撤销控制权）才能再次运行。全零 `SPEED` 不会清除闩锁。目标相对判据使 5%/10% 低速断编码器也能触发故障。
- 标定成功时先停车再写 NVS，只有保存成功才回 `CAL DONE`；保存失败保持 `valid=0` 并回 `CAL FAIL nvs`。v1 的 `CAL?` 不逐轮打印 `vsim`，只能由 `VMAX/LIMIT` 反推 `min(vsim)`。

闭环控制 10 ms tick、40 ms 测速窗口、前馈+PI；打滑仍会造成残余偏航，悬空标定的最大速度高于落地带载，软件编码器高速时可能丢边沿。完整协议、NVS 键名和限制见 [MECANUM.md](MECANUM.md)。

## 串口协议

为兼容原统一协议，真实模式标识仍为 `MOTOR_A_TEST`，四路能力与引脚版本由 CAPS 确认。上电默认 DRY_RUN，四路 IN 输出初始化为零、STBY 关闭；模拟 ARM/DRIVE 不控制电机。

```text
WHO
CAPS
MODE MOTOR_A_TEST
STATUS
ENC B
ENCZERO B
JOG B 50 1000
STOP
IO
MODE DRY_RUN

MODE CHASSIS
LAYOUT CBAD 1 1 1 1
LAYOUT?
CAL
CAL?
PID 2000 1000 80
TELEM 1
SPEED 30 0 0 2000
SPEEDW 30 30 30 30 2000
STOP
SAVE
```

- `JOG B 50 1000`：B 路 +50%、1000ms，负数反向。运行中任何新 JOG 均拒绝，不延长原定时。
- `STOP`：关闭共享 STBY，并清零全部 8 个 PWM 输出。
- `STATUS`：`STOPPED ALL` 或 `RUNNING B` 等，并回传四路编码器。面板每约 0.7 秒自动查询维持通信。
- `ENC` 读四路，`ENC B` 只读 B，格式为 `ENC B ticks=... invalid=... A=0 B=1`。
- `ENCZERO B` 停止后清 B；`ENCZERO ALL` 停止后清四路，回复 `ZEROED B/ALL`。直接 Monitor 无通道 ENCZERO 默认清 A。
- `IO` 读取共享 STBY 及四组 PWM，示例 `IO mode=MOTOR_A_TEST standby=0 A=0,0 B=0,0 C=0,0 D=0,0`。
- 直接 Monitor 旧格式 `JOG 50 1000` 兼容为 A 路。面板中省略通道则使用下拉所选通道；显式填写其他通道的 JOG 要求先切换并重新确认。
- 通过工作台顶部切换模拟/真实模式；串口框禁止直接 MODE、多行或控制字符。
- `SPEED`/`SPEEDW`/`CAL`/`PID`/`PID?`/`CAL?`/`CAL RESET`/`TELEM`/`SAVE` 仅 `MODE CHASSIS` 可用；`STATUS` 在标定中回 `RUNNING CAL`，故障时追加 `FAULT last=<reason>` 或 `FAULT none`。
- `CAPS` 的 `CHASSIS_CAPS2` 行表示闭环固件；没有该行的旧固件不能被新面板用于闭环。

运行时 1500ms 收不到 PING/STATUS 会关闭四路输出。每次停止后至少冷却 1 秒，手动 STOP 可提前结束长运行。软件停止为关闭驱动、滑行，不保证机械立即停稳。

## 验证

历史记录（2026-10 及更早，未包含轮速闭环）见 `../debug-workbench/TEST-REPORT.md`。2026-10-31 至 2026-11-01 的闭环改动已完成全部主机测试（`test_wheel_speed`、`test_calibration`、`test_config_store`、`test_closed_loop`、`test_motor`、`test_chassis`、`test_bluetooth`）、Python 离线测试和 `pio run`。实机闭环验收（悬空标定、重启回读、低速跟随、落地对比、故障与断链）尚未执行，结果待用户按 [CLOSED-LOOP-VERIFICATION.md](CLOSED-LOOP-VERIFICATION.md) 记录；在用户确认前不声称闭环性能已达标。
