# 四轮驱动与麦克纳姆控制（MECANUM_V2）

2026-11-01 新增轮速闭环（WHEEL_SPEED_V1）：`SPEED`/`SPEEDW`、`CAL`、`PID`、`TELEM`、`SAVE`。闭环需先在四轮悬空状态完成标定；开环 `MECANUM`/`WHEELS` 语义不变。新面板通过 `CHASSIS_CAPS2` 能力行识别闭环，旧面板仍按原能力行走开环。闭环协议、标定、故障和限制见下文“轮速闭环”章节，实机验收见 [CLOSED-LOOP-VERIFICATION.md](CLOSED-LOOP-VERIFICATION.md)。

2026-10-27 新增蓝牙：Windows 配对 `PushRobot-ESP32` 后，在原面板选择蓝牙传出 COM 端口即可；USB 仍可用，同一时间只允许一端控制。详见 [BLUETOOTH.md](BLUETOOTH.md)。轮位、方向和运动指令不变。

2026-10-27 当前配置：**C 左前、B 右前、A 左后、D 右后，布局 CBAD，面板四个方向全部 +1**。原来的 ADBC 位置记录已被取代。

最新逐路复测确认 B/C 逻辑正输出朝车尾，A/D 正常，因此固件仅反转 B/C。当前通道 A/B/C/D 的电气极性均为 -1；该方向修正需要烧录。轮位、面板全 +1 和 GPIO 接线保持不变。

2026-10-22 已将方向修正下移到 ESP32 底层驱动。所有 JOG、WHEELS、MECANUM 输入统一使用正数向前、负数向后；实际电气方向由 `src/motor_profile.h` 固化为通道极性，不再让面板补偿。

这是固件接口语义变更，需要同时更新固件和源码面板。旧版面板的 -1 补偿在 V2 中会被拒绝，避免重复取反；新版底盘面板也不会在 V1 固件上开放运行。电气引脚仍为 FOUR_V1，PWMA/B/C/D 固定接驱动板自身 5V，不需要改接线。

## 启动与第一次使用

源码运行时双击项目根目录 `run_debug.cmd`，在工作台顶部选择“麦克纳姆底盘”。免安装版请使用 20261022_MECANUM_V2 分享包的“启动麦克纳姆底盘.cmd”；20261021 包的旧 EXE 不含新版底盘功能。

1. 驱动电源关闭时先连接 ESP32，选择 CP210x 串口，等待 FOUR_V1 / MECANUM_V2 和停止状态确认。
2. 实物必须核对为下图 **X 型**，斜线代表从车顶看到的小滚轮轴向。用户照片有遮挡，不能把照片判断当作安装验收。L/R 可能表示滚轮旋向，需结合厂家标法与装好后的排列判断。

```text
        X 型（支持）      O 型（本版不使用）
          车头 ↑            车头 ↑
         \     /           /     \
         /     \           \     /
```

3. 固定底盘并架空车轮，确认接线与供电后，逐路填写正输出核对方向：四轮顶部都应朝车头。若换线后不符，停止并修改 `src/motor_profile.h` 的通道电气极性、重新编译烧录，不在面板叠加反向修正。
4. 本车保持默认轮位 **CBAD**、方向 **+1/+1/+1/+1**，核对 X 型后点击“应用轮位和方向”，等待设备确认。连接、切换和重启均不自动应用布局，也不自动运行。旧打包窗口可直接改四个轮位下拉框再应用，无需重烧录。
5. 确认现场条件后点击方向按钮，或输入 x/y/z 组合。持续时间支持小数秒、精确到毫秒；每次一个动作，不排队续跑，停后约 1 秒可开始下一个动作。
6. STOP / Esc、断开、关闭、切换模式或窗口失焦均请求关闭全部输出；固件自行定时，1.5 秒无有效 PING/STATUS 也会停止。

本车实测值保存为面板的默认配置；手动修改尚不保存。应用后的布局只在 ESP32 本次模式中有效，没有写入 Flash；再次连接需点击应用，不能依靠上一次连接的确认状态。

## 运动学与单位

采用标准 45°、X 型、车体坐标，x 向前、y 向左、z 逆时针；输入均为 -100～100 的归一化百分比分量。FL=x-y-z，FR=x+y+z，RL=x+y-z，RR=x-y+z。若任一结果超过 100，四轮一起同比缩放，再映射到实际通道，最后仅由统一底层驱动转换成电气方向。轮位层的四个符号固定 +1。

这是开环输出混控。z 是旋转输出分量，不是角速度；x/y 不是 mm/s。闭环 `SPEED`/`SPEEDW` 在同样的混控结果上增加逐轮编码器测速与 PI，使四轮实际转速更接近目标；它仍不是物理单位运动学，也不承诺车速、位置、旋转角度、绝对直线度或四轮绝对同步。物理单位、IMU、视觉和里程计仍未实现。原 DRY_RUN 的 mm/s 与 °/s 模拟协议保留，其目标不会传给真实输出。

参考坐标与轮位约定：[WPILib Mecanum Kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html)。本项目独立实现百分比混控，没有引入 WPILib 依赖。

## 轮速闭环（WHEEL_SPEED_V1）

2026-11-01 新增。开环 `MECANUM`/`WHEELS` 仍是默认的百分比混控；闭环只在 `SPEED`/`SPEEDW` 动作期间生效，不改变开环语义。闭环需要先在四轮悬空状态完成标定；未标定、标定失效或故障闭锁时，闭环运动命令一律被拒绝。

### 控制结构

- 控制周期 10 ms；编码器计数差分经 40 ms（4 样本）滑动窗口得到轮速 `counts/s`。
- 每轮“两点前馈 + PI + 抗积分饱和”：前馈由标定的 `a + b×|v|` 给出，PI 在归一化速度误差上工作，输出逻辑占空比 ±100%。
- 目标斜坡 `AMAX=300%/s`；目标为零、目标符号改变、STOP、故障、标定开始或控制 tick 间隔超过 50 ms 时清零积分。
- `Vmax = (LIMIT/100) × min(vsim_i)`；单轮目标 = 混控百分比 / 100 × Vmax。默认 `Kp=2.0`、`Ki=1.0`、`LIMIT=80`、`AMAX=300`。
- 每个非零轮目标必须 ≥ 5% Vmax；出现 `0 < |p| < 5` 的轮子时整条命令拒绝（`ERROR closed-loop target too low`）。全零目标等同 STOP。
- 闭环仍是一次一个定时动作：运行中拒绝新动作，停止后约 1 秒冷却，1.5 秒无 PING/STATUS 仍会停车。

### 命令（仅 MODE CHASSIS）

| 命令 | 说明 |
|---|---|
| `SPEED x y z ms` | 闭环麦轮动作；x/y/z 为 -100..100 百分比分量，复用 X 型混控；需先标定并确认布局 |
| `SPEEDW A B C D ms` | 闭环逐轮直控，每轮 -100..100；用于调参或单轮验证 |
| `PID <kp_milli> <ki_milli> <limit>` | 设置 Kp/Ki（千分之一，0..10000）和速度上限（10..100）；仅停止时允许；标定有效时，limit 变化立即按已存 vsim 重算 Vmax |
| `PID?` | 回读 `PID_OK KP=... KI=... LIMIT=...` |
| `CAL` | 启动四轮悬空标定；仅停止、布局已确认、无故障时允许 |
| `CAL?` | 回读 `CAL VALID=... VMAX=... LIMIT=...` 和四行 `CALW ...`；故障闭锁期间也可读 |
| `CAL RESET` | 仅清除标定有效性（保留数值供诊断）；仅停止且无故障时允许；不解除故障闭锁 |
| `TELEM 0\|1` | 开/关 20 Hz `TLM` 遥测 |
| `SAVE` | 把当前标定与 PID 参数写入 NVS；仅停止时允许 |

`CAPS` 在原有能力行之外新增：

```text
CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI
```

新面板必须看到该行才开放闭环控件；旧面板只认旧 `CHASSIS_CAPS`，继续走开环。`PID`/`PID?`/`TELEM`/`SAVE`/`CAL`/`CAL?`/`CAL RESET` 在非 `MODE CHASSIS` 下由统一入口拒绝并回 `ERROR requires MODE CHASSIS`；`SPEED`/`SPEEDW` 只在 `MODE CHASSIS` 分支中解析，其它模式下不会被当作底盘闭环命令执行（会落入该模式的解析器并报该模式的错误）。

### 标定步骤

前提：`MODE CHASSIS`、X 型布局已应用并回读确认、底盘停止、无故障闭锁、控制权在当前端口；面板还要单独勾选“场地已清空…”并在弹窗中确认四轮悬空。标定会让轮子以 100% 占空比转动，**必须架空**。

1. 发送 `CAL`；固件立即把 `valid` 清零（RAM 与 NVS），旧标定马上失效；回复 `CAL START`。
2. 对 A/B/C/D 逐轮：停 200 ms → 60% 占空比 1.2 s，取最后 500 ms 测速；若 |v| < 50 counts/s，停 200 ms 后改用 75% 重试一次；仍无速度则 `CAL FAIL no_feedback <i>` 并中止。
3. 记录编码器符号 `sign` 和低速点 `v60`；随后该轮 100% 1.2 s，取最后 500 ms 得 `v100`；由 `b = (100-d1)/(v100-v60)`、`a = d1 - b×v60` 拟合前馈线；`d1` 为本次实际使用的 60% 或 75%。失败时 `CAL FAIL bad_line <i>`。
4. 四轮同时 100% 1.5 s，取最后 500 ms 得 `vsim_i`；任一路 < 200 counts/s 则 `CAL FAIL bad_sim <i>`。
5. `Vmax = LIMIT% × min(vsim_i)`，标记 `valid=1` 并自动写 NVS，回复 `CAL DONE VALID=1 VMAX=...`。整个流程约 12–15 s（60% 重试时更长）。
6. 成功后同样执行一次停车，对外部运动命令建立 1 秒冷却。失败或中止时保持 `valid=0`，闭环禁用，必须排查后重新标定；v1 不做逐轮降级。
7. `CAL?` 示例：

```text
CAL VALID=1 VMAX=2560.0 LIMIT=80
CALW A SIGN=1 V100=4250.0 DEADBAND=32.5 SLOPE=0.0160
...
```

`DEADBAND` 即前馈截距 `a`，`SLOPE` 即 `b`。v1 不逐轮打印 `vsim`；只能由 `VMAX` 与 `LIMIT` 反推最小值 `min(vsim) = VMAX × 100 / LIMIT`，或从标定过程的 `CAL PROGRESS speed` 记录 `v60/v100`。

标定期间：运动命令和 `ENCZERO` 被拒绝；`STATUS` 回 `RUNNING CAL`；`STOP`、模式切换、断链/超时、控制权撤销都会中止标定并停车，不保存新标定。

### 故障与闭锁

每个控制 tick 对四轮分别检查；任一条件持续满足即停车、清输出并置故障闩锁：

| 故障 | 判据 | 持续时间 |
|---|---|---|
| `encoder_sign` | `abs(u) ≥ 50`、`u×v < 0`、`abs(v) ≥ 0.03×Vmax` | 300 ms |
| `no_feedback` | 绝对：`abs(u) ≥ 85`、`abs(v) < 0.02×Vmax`；或目标相对：`abs(vref) > 0`、`abs(u) ≥ 50`、`abs(v) < 0.20×abs(vref)`（同时满足 `stall` 时交给 `stall`） | 800 ms |
| `stall` | `abs(vref) ≥ 0.20×Vmax`、`abs(u) ≥ 98`、`abs(v) < 0.20×abs(vref)` | 1000 ms |

- 判据以实际输出占空比 `u` 为主，所以低速命令下编码器断开、线松或轮子卡死同样会触发 `no_feedback`。目标相对判据专门覆盖 5%/10% 目标：此时 PI 通常只把占空比推到 75–85%，达不到绝对判据的 85% 门槛。
- 触发时回 `FAULT <reason>`，`STATUS` 追加 `FAULT last=<reason>` 或 `FAULT none`；`SPEED`/`SPEEDW`/`CAL` 被拒绝并提示 `ERROR fault latched; send STOP`。
- 闩锁只能由显式 `STOP`、切换模式（或断链/超时撤销控制权）解除；`SPEED 0 0 0` 虽然会停车，但**不会**清除闩锁。`PID`/`PID?`/`CAL?`/`TELEM`/`SAVE` 在闭锁期间仍可用；`CAL RESET` 要求先 `STOP`。
- 故障后不会自动重试；必须排查接线/编码器/机械卡阻，再 `STOP` 后重新运行。

### 遥测

`TELEM 1` 后以 20 Hz（50 ms）向当前控制端口发送一行，共 13 个数值字段：

```text
TLM <ms> <rA> <rB> <rC> <rD> <sA> <sB> <sC> <sD> <uA> <uB> <uC> <uD>
```

依次为时间戳、四轮目标 counts/s、四轮实际 counts/s、四轮逻辑占空比 %。STOP 后仍可能继续发送，但报文是 `resetClosedLoop` 清零后的目标/实际/占空比（四个占空比为 0），**不是**停车瞬态；标定期间不发送。`TELEM 0` 关闭；切换模式或撤销控制权时固件自动关闭。面板在 STOP、断开、失焦后也会发 `TELEM 0`，并需要新的用户动作才恢复；`TLM` 只刷新“闭环调参”页表格，不写入日志。USB 发送前会检查 `Serial.availableForWrite()`，容不下整行时跳过，避免遥测阻塞 10 ms 控制 tick；蓝牙仍走 16 包发送队列。

### NVS 与版本

命名空间 `chassis_cl`，结构版本键 `ver` 为 float，当前值 `1`。实际键名：

- `valid`（uint8）
- 每轮：`s0..s3`（编码器符号 ±1）、`a0..a3`、`b0..b3`、`v100_0..v100_3`、`vsim0..vsim3`
- `vmax`、`kp`、`ki`、`limit`、`amax`

写入顺序为 `valid=0` → 四轮与 `vmax` → 最终 `valid` → 最后 `ver`；中途写失败不会留下可用的 `valid=1` 记录。标定成功时**先停车、再写 NVS**，`CAL DONE` 在保存成功后才会发送；保存失败则保持 `valid=0` 并回 `CAL FAIL nvs`。`load` 只接受 `valid==1`（其它非零值视为损坏），要求 `ver==1`，校验符号/参数范围（`kp/ki≤10`、`amax≤1000`，每轮 `vsim/v100≥200`），并用已存 `vsim` 与当前 `limit` **重新计算** `vmax`（NVS 里的 `vmax` 仅是缓存）。`CAL` 成功自动保存标定；`PID` 只改 RAM，需 `SAVE` 才持久化；`CAL RESET` 只清 `valid`，保留数值供诊断，NVS 写失败会回 `ERROR NVS save failed`。

**残余风险（NVS 完全不可写）：** 如果已有 `valid=1` 记录，而 `CAL` 开始时第一次写 `valid=0` 就失败，本次开机的 RAM 已标记 `valid=false` 并拒绝 `CAL`/`SPEED`，但 Flash 里的旧记录无法被覆盖；重新上电后旧标定可能再次加载为 `VALID=1`。遇到 `CAL FAIL nvs` 应视为 NVS 异常：停止使用闭环，检查/擦除 NVS（重新烧录或擦除 Flash）并重启确认 `CAL?` 为 `VALID=0` 后再重新标定。

### 已知限制

- **打滑无法由轮速环根治。** 编码器测到的是轮子转速，不是车体速度；右前/左后打滑时即使四轮转速相等，车仍会偏。轮速闭环能减小“同占空比不同转速”造成的偏航，不能保证直线或航向。
- 标定在悬空状态完成；落地负载、地面摩擦和电池压降会让实际最大速度低于标定值。`LIMIT=80` 留了调节余量，PI 会补偿；四轮同时饱和时只能降速。必要时降低 `LIMIT` 或重新标定。
- 编码器使用软件 CHANGE 中断四倍频；高速时四轮合计约 2.2 万边沿/s，蓝牙开启时可能偶发丢边沿。`invalid` 计数和速度一致性可用于发现；严重时后续迁移 PCNT。
- 前馈是两点线性近似，正反向死区假设对称，低速和换向附近误差较大；依赖 PI 修正。
- 没有物理单位运动学、IMU、视觉、里程计或电流/力矩闭环；不承诺 mm/s、位置、旋转角度、绝对直线度或四轮绝对同步。
- 不支持 `0 < |target| < 5%` 的非零闭环轮目标；某个轮目标为 0 是允许的（如纯横移/旋转混流）。不支持运行中流式改速。

## 固件串口接口

- `MODE CHASSIS`：先关闭全部输出，进入底盘模式并撤销已确认布局。
- `CAPS`：保留 `CAPS MOTORS=ABCD PINS=FOUR_V1`，另回 `CHASSIS_CAPS API=MECANUM_V2 MIX=X UNITS=PERCENT FORWARD=UNIFIED`；闭环固件再回 `CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI`。
- `POLARITY`：只读回显最新通道电气极性 `MOTOR_FORWARD A=-1 B=-1 C=-1 D=-1`，不运行电机。
- `LAYOUT CBAD 1 1 1 1`：本车轮位配置，方向全部 +1；旧版的负号补偿命令将被拒绝。仅停止时可改，应用参数不会运行电机。
- `LAYOUT?`：回读布局与 CONFIGURED 状态。
- `WHEELS 50 -50 0 25 500`：A/B/C/D 分别输出 +50/-50/0/+25%，运行 500ms，与轮位混控无关。
- `WHEELS 50 50 50 50 500`：四轮共同前进；`WHEELS -50 -50 -50 -50 500`：四轮共同后退。这里是车体前进逻辑，已不再暴露原始电气正负。
- `MECANUM 50 0 0 500`：向前输出分量 50%，500ms；必须先确认布局。
- `MECANUM 0 50 0 500`：左移；`MECANUM 0 0 50 500`：逆时针；二者符号相反即反向。
- `MECANUM 50 50 0 500`：左前斜移；全零向量等同 STOP。
- `STOP`：关闭共享 STBY 与全部 8 路 PWM；`STATUS`、`PING`、`ENC`、`IO` 沿用原格式，运行状态为 `RUNNING CHASSIS`（标定中为 `RUNNING CAL`）。
- 闭环命令 `SPEED`/`SPEEDW`/`PID`/`PID?`/`CAL`/`CAL?`/`CAL RESET`/`TELEM`/`SAVE` 仅 `MODE CHASSIS` 可用，详见上文“轮速闭环”章节；开环 `MECANUM`/`WHEELS` 行为不变。

时间范围为 1～2147483647ms。运行中新的运动指令会被拒绝，不延长原动作；不是高频流式速度接口。界面使用同一串口连接和后台心跳，不能同时开其他串口助手。真实模式和 DRY_RUN 仍严格隔离。

## 主要文件

- `src/mecanum.h`：无硬件依赖的混控、轮位映射和符号设置。
- `src/motor_profile.h`：本车电气极性标定；这是唯一的电气方向转换位置。编码器计数仍保留原始相位符号，正负由标定得到的 `sign` 修正。
- `src/wheel_speed.h`：测速估计、前馈+PI、抗积分饱和和逐轮故障判据。
- `src/calibration.h`：两点前馈线拟合与 Vmax 计算。
- `src/config_store.h`：`chassis_cl` NVS 读写、版本与参数校验。
- `src/main.cpp`：四路输出、底盘协议、闭环控制 tick、标定状态机、遥测、模式隔离、定时与通信停止。
- `chassis_link.py`：握手、布局回读确认、边界检查、闭环命令、CAL/TLM/FAULT 解析、STOP 优先队列。
- `../debug-workbench/chassis_app.py`：布局、方向键、组合输出、闭环开关、闭环调参页、编码器与日志。
- `tests/test_chassis.cpp`、`tests/test_closed_loop.cpp`、`tests/test_chassis_link.py`、`../debug-workbench/tests/test_chassis_ui.py`：离线回归。
- `CLOSED-LOOP-VERIFICATION.md`：实机验收 8 步清单。

实物检查工具 `../debug-workbench/tests/verify_chassis_device.py --port COM4 --gui` 只查询/切换/停止，不发送真实运动指令；不应将静止 IO 检查当作电机运动或电气测量验收。
