# 轮速闭环实机验收清单（WHEEL_SPEED_V1）

状态：**离线回归已通过（2026-11-01）；实机结果待用户执行，尚未确认。**
在用户完成本清单并签字前，不得声称轮速闭环性能已达标，也不得把离线测试通过等同于实车可用。

适用范围：`motor-bringup` 固件（`CHASSIS_CAPS2 API=WHEEL_SPEED_V1`）与 `debug-workbench` 底盘面板。开环 `MECANUM`/`WHEELS` 不在本清单的闭环性能判定内，但仍需在步骤 4/5 中作为对照。

## 安全前提

- 标定与悬空测试必须把**四轮完全架空**，周围无障碍物、无人手靠近轮子。标定会以 60%/75%/100% 占空比转动，落地会跑车。
- 落地测试前清空场地，系好电池与线缆，保留至少 2 m 直线空间；操作者手边可随时切断驱动电源。
- 任何异常（异味、异响、轮子持续输出、面板无响应）先断驱动电源，再排查；不要只依赖软件 STOP。
- 堵转测试会短时大电流，单次不超过约 2 s；不要用手直接阻挡旋转轮。
- **测 1.5 s 通信超时必须保持 ESP32 供电。** 不要拔 USB 线来测超时。断链测试使用“强杀面板进程”或“保持串口打开但停止心跳”的方式。
- **切断驱动电池（ESP32 仍由 USB 供电）：** 固件逻辑仍在运行，PWM/STBY 状态仍有定义，但电机失去转矩。这验证的是动力断电/急停，不是 1.5 s 超时。
- **拔掉 ESP32 USB（逻辑断电）：** 固件停止运行，PWM/STBY 不再受控，电机输出未定义。这也不是 1.5 s 超时。
- 以上断电/急停测试必须在四轮架空或车辆可靠约束时进行；不能声称“1.5 s 内停车”。

## 工具与通信（先读）

底盘面板**没有任意命令输入框**；`ChassisLink.send_line()` 只允许查询和 `STOP`，运动只能通过受控按钮/方法发送。面板在故障闭锁时会禁用运动按钮，**断开面板会发送 `STOP` 并清除故障闩锁**。因此：

| 用途 | 工具 | 说明 |
|---|---|---|
| 标定、PID、正常运动/遥测 | 底盘面板 | 面板每约 0.7 s 发送 `STATUS` 维持 1.5 s 心跳；标定有独立确认框 |
| `SPEED`/`SPEEDW` 原始命令、`TELEM`、`CAL?` 原样输出、全零 `SPEED`、故障闩锁拒绝、链路静默测试 | 串口脚本 | 必须自己持有 `MODE CHASSIS`，并每 <1.5 s 发送 `PING`/`STATUS`；脚本必须持续读串口 |

操作规则：

1. **同一时间只允许一个控制端。** 从面板切到脚本前，先在面板点“断开”（正常断开；它会发 `STOP`，但标定结果仍保存在 NVS）。不要在故障闩锁期间断开面板——那会发 `STOP` 并清除闩锁，导致无法验证拒绝行为。
2. 面板的“场地已清空，车轮/电机已固定，可开始本次测试”必须勾选，才能启用“开始标定”；点击后还会弹出独立的“四轮悬空、周围无障碍、电池电压正常”确认框。两个确认都不能省略。
3. 脚本用于 `SPEED` 混合动作时需要布局；脚本用 `--layout CBAD` 重新应用布局。使用前必须已在步骤 1 完成 X 型实物核对。脚本用于 `CAL?`/`PID?` 查询或 `SPEEDW` 直控时可省略 `--layout`。
4. 故障测试期间保持脚本运行并持续心跳；不要在故障后关闭串口/面板，直到发送显式 `STOP` 并记录 `FAULT none`。

### 串口脚本（保存到 `%TEMP%\chassis_raw.py`）

在 PowerShell 中执行以下命令创建脚本（内容已按本清单要求的控制权/心跳逻辑编写）：

```powershell
@'
"""Own MODE CHASSIS, keep a <1.5 s PING heartbeat, and send raw chassis commands."""
import argparse
import sys
import threading
import time

import serial

write_lock = threading.Lock()


def send(ser, text):
    """All writers (main thread and heartbeat) share one lock."""
    with write_lock:
        ser.write(text.encode())
        ser.flush()


def reader(ser, stop):
    while not stop.is_set():
        line = ser.readline()
        if line:
            print(line.decode("utf-8", errors="replace").rstrip(), flush=True)


def heartbeat(ser, stop, pause):
    while not stop.is_set():
        if not pause.is_set():
            send(ser, "PING\r\n")
        time.sleep(0.4)


def wait_for(ser, needle, timeout, abort_on_error=True):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline()
        if not line:
            continue
        text = line.decode("utf-8", errors="replace").rstrip()
        print(text, flush=True)
        if needle in text:
            return text
        if abort_on_error and text.startswith("ERROR "):
            raise RuntimeError(text)
    raise RuntimeError("timeout waiting for " + needle)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--layout", default=None, help="e.g. CBAD; sends LAYOUT <value> 1 1 1 1")
    parser.add_argument("--link-timeout", action="store_true", help="send SPEED, wait for SPEED_RUN, pause heartbeat, wait for STOPPED link_timeout")
    args = parser.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(1.0)
    stop, pause = threading.Event(), threading.Event()
    send(ser, "MODE CHASSIS\r\n")
    time.sleep(0.5)
    if args.layout:
        send(ser, f"LAYOUT {args.layout} 1 1 1 1\r\n")
        time.sleep(0.3)
    try:
        if args.link_timeout:
            # Do not start the test window until the firmware acknowledged the
            # motion AND the operator saw the wheels spin. The line alone is
            # necessary but not sufficient evidence of a real run.
            send(ser, "SPEED 50 0 0 5000\r\n")
            wait_for(ser, "SPEED_RUN", 2.0)
            threading.Thread(target=heartbeat, args=(ser, stop, pause), daemon=True).start()
            time.sleep(0.5)  # visible motion + normal heartbeats before silence
            pause.set()
            print("TEST WINDOW START", flush=True)
            started = time.monotonic()
            wait_for(ser, "STOPPED link_timeout", 3.0)
            print(f"[elapsed {time.monotonic() - started:.2f} s after heartbeat pause]", flush=True)
        else:
            threading.Thread(target=reader, args=(ser, stop), daemon=True).start()
            threading.Thread(target=heartbeat, args=(ser, stop, pause), daemon=True).start()
            print("raw> ", end="", flush=True)
            for line in sys.stdin:
                command = line.strip()
                if not command:
                    print("raw> ", end="", flush=True)
                    continue
                if command.lower() in ("exit", "quit", "!quit"):
                    break
                send(ser, command + "\r\n")
                print("raw> ", end="", flush=True)
    finally:
        pause.clear()
        stop.set()
        time.sleep(0.1)
        try:
            send(ser, "STOP\r\n")
            time.sleep(0.2)
        except serial.SerialException:
            pass
        ser.close()


if __name__ == "__main__":
    main()
'@ | Set-Content -Encoding utf8 "$env:TEMP\chassis_raw.py"
```

启动交互脚本（每 0.4 s 自动 `PING`；输入 `exit` 退出并发送 `STOP`）：

```powershell
& 'D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe' "$env:TEMP\chassis_raw.py" --port COM4 --layout CBAD
```

链路静默测试（脚本发出 `SPEED 50 0 0 5000` 后暂停心跳 3 s，但仍持续读串口）：

```powershell
& 'D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe' "$env:TEMP\chassis_raw.py" --port COM4 --layout CBAD --link-timeout
```

## 记录准备

| 项目 | 记录值 |
|---|---|
| 日期 / 操作者 | |
| 固件来源与编译时间 | |
| `CAPS` 完整输出（含 `CHASSIS_CAPS2`） | |
| 面板版本 / 启动方式 | |
| 端口（USB / 蓝牙 COM） | |
| 电池标称与实测电压 | |
| 场地材质 / 负载 | |

固件烧录（在 `motor-bringup` 目录；若 `pio` 不在 PATH，使用 `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`）：

```powershell
cd D:\Users\1713810750\推物块机器人\motor-bringup
pio run -t upload --upload-port COM4
```

烧录后确认面板/脚本读到 `CAPS` 同时返回旧 `CHASSIS_CAPS` 行和：

```text
CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI
```

---

## 步骤 1：烧录、悬空标定并记录四轮参数

**工具：面板完成标定；串口脚本读取 `CAL?` 原始输出。**
**目的**：确认闭环固件可烧录、标定流程完整、四轮符号/量程/前馈线合理。

操作：

1. 断开驱动电源，按 `FOUR-MOTOR-WIRING.md` 核对接线；四轮架空，电池电压正常。
2. 连接面板，选择端口；在“麦克纳姆控制”页核对实物 X 型、应用轮位，等待 `LAYOUT` 回读确认。
3. 勾选“场地已清空，车轮/电机已固定，可开始本次测试”，确保“开始标定”按钮变为可用（面板要求该确认 + X 型 + 已应用布局）。
4. 在“闭环调参”页点“开始标定”，在弹窗中确认“四轮悬空、周围无障碍、电池电压正常”；保持手离开轮子，等待约 12–15 s（60% 不起转重试 75% 时更长）。
5. 从面板日志记录进度行（面板会显示/记录 `CAL PROGRESS`）：

```text
CAL START
CAL PROGRESS run <0..3> <60|75>
CAL PROGRESS speed <0..3> <v60>
CAL PROGRESS line <0..3> a=<a> b=<b>
...
CAL DONE VALID=1 VMAX=<vmax>
```

6. 标定完成后，面板状态行显示 `VMAX`/`LIMIT`。正常点击“断开”（会发 `STOP`；标定已保存，不受影响）。
7. 运行串口脚本（查询不需要布局）：

```powershell
& 'D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe' "$env:TEMP\chassis_raw.py" --port COM4
```

在 `raw>` 提示符输入 `CAL?`，记录原样输出：

```text
CAL VALID=1 VMAX=<vmax> LIMIT=<limit>
CALW A SIGN=<±1> V100=<v100> DEADBAND=<a> SLOPE=<b>
CALW B ...
CALW C ...
CALW D ...
```

输入 `exit` 退出脚本（自动发 `STOP`）。

判定与记录：

| 轮（通道） | SIGN | V100 counts/s | DEADBAND a（%） | SLOPE b（%/counts/s） | CAL PROGRESS 的 v60 |
|---|---|---|---|---|---|
| A（左后 RL） | | | | | |
| B（右前 FR） | | | | | |
| C（左前 FL） | | | | | |
| D（右后 RR） | | | | | |

- `CAL DONE VALID=1`；`VMAX > 0`。
- 每轮 `V100 > v60`、`V100 ≥ 200`、`a ∈ [0, 90)`、`b > 0`。
- v1 的 `CAL?` **不逐轮打印 `vsim`**；只能记录 `min(vsim) = VMAX × 100 / LIMIT`（counts/s），并由 `CALW` 记录每轮 `V100` 和前馈 `a/b`。不要声称已取得逐轮 `vsim`。
- 标定期间无 `FAULT`，结束后四轮停转，约 1 s 内新的运动命令会被冷却拒绝。

失败处理：任一路 `CAL FAIL no_feedback <i>` / `bad_line <i>` / `bad_sim <i>` / `nvs` 时，记录原因并停止，不要进入后续步骤；`valid` 已为 0，闭环禁用。排查接线、编码器、供电、机械卡阻后重新标定。

**用户结果（待填）**：通过 / 不通过；备注：

---

## 步骤 2：断电重启并回读 NVS

**工具：串口脚本（`CAL?`/`PID?` 原样输出）。**
**目的**：确认标定掉电保存，重启后无需重新标定。

操作：

1. 断开 ESP32 电源（或按 EN/复位）后重新上电；固件默认回到 `DRY_RUN`，布局不会自动恢复，遥测默认关闭。
2. 运行串口脚本（查询不需要布局）：

```powershell
& 'D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe' "$env:TEMP\chassis_raw.py" --port COM4
```

在 `raw>` 提示符依次输入 `CAL?`、`PID?`，记录原样输出后输入 `exit`。

判定与记录：

- `CAL VALID=1`，`VMAX` 与步骤 1 一致；`LIMIT` 与保存前一致。
- 四行 `CALW` 的 `SIGN/V100/DEADBAND/SLOPE` 与步骤 1 记录一致（允许显示精度差异）。
- `PID?` 返回 `PID_OK KP=... KI=... LIMIT=...`；若步骤 1 后执行过 `SAVE`（面板“保存参数”），应与保存值一致；否则记录当前值。
- 若 `CAL VALID=0`：NVS 未保存或版本/参数校验失败，记录现象并停止；不要继续闭环测试。

**用户结果（待填）**：通过 / 不通过；`CAL?` 原始输出：

---

## 步骤 3：悬空低速跟随与最小目标

**工具：面板做运动/遥测；串口脚本做原始 `SPEED` 下限与全零检查。**
**目的**：确认四轮目标/实际/占空比可读、方向正确、低速稳态误差可接受。

操作（四轮架空）：

1. 面板连接并应用布局；勾选“场地已清空，车轮/电机已固定，可开始本次测试”（连接/STOP 会清除该确认）；再勾选“轮速闭环”（需 `CAL VALID=1`），在“闭环调参”页激活遥测。
2. 力度填 `30`，持续时间 `2` 秒，点“前进 ↑”。面板发送 `SPEED 30 0 0 2000` 并维持心跳；观察 `TLM` 表格约 1.5 s，记录最后约 0.5 s 的四轮目标、实际、占空比。停止后等待约 1 s，并**重新勾选现场确认**（`STOP`/定时停止会清除）。
3. 力度改为 `60`，先确认现场确认仍勾选，再重复一次 `SPEED 60 0 0 2000`，记录 60% 工况。
4. 正常断开面板（发送 `STOP`），运行串口脚本：

```powershell
& 'D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe' "$env:TEMP\chassis_raw.py" --port COM4 --layout CBAD
```

在 `raw>` 提示符依次输入：

```text
SPEED 4 0 0 1000
SPEED 0 0 0 1000
exit
```

预期：`SPEED 4 ...` 返回 `ERROR closed-loop target too low`；`SPEED 0 0 0 ...` 返回 `STOPPED command` 并停车。记录后退出脚本。

说明：`SPEED 30` 的目标斜坡约 0.1 s 到达（`AMAX=300%/s` 时满量程约 0.33 s）。面板不会对 x/y/z 合成做 5% 预检，混控后出现 `0 < |target| < 5%` 的轮目标时由固件拒绝；某个轮目标恰好为 0 是合法的。

记录（30% / 60% 各一份）：

| 轮（通道） | 目标 counts/s | 实际 counts/s | 误差 % = (实际−目标)/目标×100 | 占空比 % | 是否振荡/异响 |
|---|---|---|---|---|---|
| A（左后 RL） | | | | | |
| B（右前 FR） | | | | | |
| C（左前 FL） | | | | | |
| D（右后 RR） | | | | | |

说明：STOP/定时停止后仍可能收到 `TLM`，但报文里是 `resetClosedLoop` 清零后的目标/实际/占空比（四个占空比为 0），不是停车瞬态；不要把 STOP 后的 TLM 当作制动过程数据。

判定：

- 四轮方向一致，均为正目标/正实际；无 `FAULT`。
- 占空比未长期贴 100%，实际速度能跟随目标；建议先以稳态误差 ≤ ±5% 为目标，具体由用户按机构与地面确认。
- 若某轮实际为负或明显不跟随：立即 `STOP`，检查编码器符号、接线、机械卡阻，不要继续提速。

**用户结果（待填）**：通过 / 不通过；备注：

---

## 步骤 4：落地直行，开环 vs 闭环对比

**工具：面板（开/闭环运动、编码器表、遥测）。**
**目的**：确认闭环能减小“同占空比不同转速”造成的直行偏差；打滑残余偏航不作为失败判据。

操作：

1. 场地画 2 m 直线与中线；电池电压记录在“记录准备”表。车辆对准中线，前后留安全距离。
2. 面板连接、应用布局；勾选“场地已清空，车轮/电机已固定，可开始本次测试”；取消勾选“轮速闭环”做开环，至少 3 次，每次间隔 >1 s：力度 `30`、时间 `2` 秒、点“前进 ↑”（发送 `MECANUM 30 0 0 2000`）。**每次 STOP/定时停止后重新勾选现场确认**。
3. 勾选“轮速闭环”（需 `CAL VALID=1`；现场确认需重新勾选），同样力度/时间做闭环，至少 3 次：发送 `SPEED 30 0 0 2000`。
4. 每次运行前后从面板“编码器与日志”页的累计计数/表格记录四轮计数（或记录 `ENC` 显示），计算四轮计数差（max−min）与均值；用卷尺/地面标记记录终点横向偏移和车头偏角。

记录：

| 工况 | 次数 | 横向偏移 mm | 车头偏角 ° | 四轮计数差 / 均值 % | 备注（打滑轮） |
|---|---|---|---|---|---|
| 开环 `MECANUM 30` | 1 | | | | |
| 开环 `MECANUM 30` | 2 | | | | |
| 开环 `MECANUM 30` | 3 | | | | |
| 闭环 `SPEED 30` | 1 | | | | |
| 闭环 `SPEED 30` | 2 | | | | |
| 闭环 `SPEED 30` | 3 | | | | |

判定：

- 闭环的四轮计数差应小于开环（这是轮速环能直接改善的部分）。
- 闭环横向偏移/偏角不差于开环，理想情况更小；由于打滑，**不要求为零**。右前/左后打滑造成的残余偏航属于已知限制。
- 若闭环明显更差：记录现象，检查标定符号、`VMAX/LIMIT`、轮胎/地面，不要用提高 PID 增益掩盖机械问题。

**用户结果（待填）**：通过 / 不通过；备注：

---

## 步骤 5：横移、斜移、旋转

**工具：面板（自定义 x/y/z 或方向按钮；开/闭环切换）。**
**目的**：确认闭环在非直行混控下方向正确、比例更接近运动学目标。

操作（每次间隔 >1 s；先开环后闭环，同一力度/时长；面板“闭环调参”页激活时可看遥测）：

1. 开环：勾选“场地已清空，车轮/电机已固定，可开始本次测试”（每次 STOP/重连后都要重新勾选），取消“轮速闭环”，力度 `30`、时间 `2` 秒，在自定义 x/y/z 分别填：
   - 左移：x=0，y=30，z=0（`MECANUM 0 30 0 2000`）
   - 左前斜移：x=30，y=30，z=0（`MECANUM 30 30 0 2000`）
   - 逆时针旋转：x=0，y=0，z=30（`MECANUM 0 0 30 2000`）
2. 闭环：重新勾选现场确认，勾选“轮速闭环”，同样填写后运行（分别发送 `SPEED 0 30 0 2000`、`SPEED 30 30 0 2000`、`SPEED 0 0 30 2000`）；每次停止后重新勾选现场确认。
3. 记录方向偏差/旋转角、实际位移、打滑轮位。

记录：

| 动作 | 工况 | 方向偏差 ° / 旋转角 ° | 实际位移/转角 | 是否打滑 | 备注 |
|---|---|---|---|---|---|
| 左移 | 开环 | | | | |
| 左移 | 闭环 | | | | |
| 左前斜移 | 开环 | | | | |
| 左前斜移 | 闭环 | | | | |
| 逆时针旋转 | 开环 | | | | |
| 逆时针旋转 | 闭环 | | | | |

判定：

- 方向与符号正确；闭环方向偏差不差于开环，理想情况更小；旋转方向不相反。
- 打滑导致的残余方向偏差属于已知限制，不作为本版失败判据；记录具体轮位和地面条件。

**用户结果（待填）**：通过 / 不通过；备注：

---

## 步骤 6：编码器故障、堵转与故障闭锁

**工具：串口脚本（固件闩锁/拒绝行为）；面板单独做故障提示 UI 检查。**
**目的**：确认 `encoder_sign` / `no_feedback` / `stall` 能停车并闭锁，且只有显式 `STOP`（或切换模式/撤销控制权）能解除。

**为什么用脚本**：面板在故障闭锁时禁用运动按钮，无法发送 `SPEED` 验证拒绝；断开面板又会发 `STOP` 并清除闩锁。必须用持续持有控制权、持续心跳的脚本完成编码器断开、低速断编码器和堵转测试。

操作（四轮架空或可靠垫起）：

1. 运行脚本（`SPEED` 混合需要布局；使用前已完成步骤 1 的 X 型核对）：

```powershell
& 'D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe' "$env:TEMP\chassis_raw.py" --port COM4 --layout CBAD
```

2. **编码器断开**：在 `raw>` 输入 `TELEM 1`，再输入 `SPEED 30 0 0 3000`；运行中拔掉任一路编码器信号线（或断开该路编码器供电）。等待最多约 3 s，记录脚本打印的故障行：

```text
FAULT no_feedback        （或 encoder_sign / stall，按实际）
```

2b. **低速断编码器（5% 目标）**：保持该路编码器断开，等待 >1 s 后输入 `SPEED 5 0 0 4000`（或 `SPEEDW 5 5 5 5 4000`）。目标相对判据应在约 1–2 s 内触发 `FAULT no_feedback` 并停车；记录实际时间和原因。输入 `STOP` 清除闩锁。
2c. **低速断编码器（10% 目标）**：同上输入 `SPEED 10 0 0 4000`；预期约 1 s 内 `FAULT no_feedback`。输入 `STOP` 清除闩锁。这两步专门覆盖“低速时 PI 只把占空比推到 75–80%、旧 85% 绝对门槛不触发”的缺陷。

3. 输入 `STATUS`；应仍为 `STOPPED ALL`，并追加 `FAULT last=<reason>`。四轮应立即停转。
4. 验证闭锁：输入 `SPEED 30 0 0 1000`，预期 `ERROR fault latched; send STOP`，不输出。
5. 验证 `CAL?` 在闭锁下仍可读（只读查询）：输入 `CAL?`，应返回 `CAL VALID=...` 与 `CALW`，不改变状态。
6. 验证全零 `SPEED` 不解除固件闩锁：

```text
SPEED 0 0 0 1000
SPEED 30 0 0 1000
```

预期：第一条回 `STOPPED command`；第二条仍被 `ERROR fault latched; send STOP` 拒绝。然后输入显式：

```text
STOP
STATUS
```

预期：`STOPPED command`；`STATUS` 显示 `FAULT none`。重新接好编码器，等待 >1 s 后可再次 `SPEED`。验证完成后输入 `exit`（脚本会再发一次 `STOP`）。

6b. **模式切换/撤销控制权清除闩锁（最终验收项，不替代显式 STOP）**：再次触发任一故障，然后依次验证两条路径并记录 `STATUS`：
   - 模式切换：发送 `MODE DRY_RUN`，再 `MODE CHASSIS`、`LAYOUT CBAD 1 1 1 1`；`STATUS` 应为 `STOPPED ALL` 且 `FAULT none`。
   - 撤销控制权：故障后停止发送 PING/STATUS >1.5 s（或将 7A 的链路静默流程用于故障状态），等待 `STOPPED link_timeout`；重新 `MODE CHASSIS`、应用布局后 `STATUS` 应为 `FAULT none`。
   - 日常操作仍要求先显式 `STOP`；这两条只用于验收“切换/撤销会清闩锁”的固件行为。

7. **堵转**：在安全前提下短时阻止一路轮子转动（机械挡块/软夹，不用手），输入 `SPEED 30 0 0 2000`。记录实际触发的是 `stall` 还是 `no_feedback`（当占空比先到 85% 而速度仍低时，`no_feedback` 可能先到），以及从阻挡到停车的时间。单次不超过约 2 s，避免驱动/电机过热。随后按第 6 步的 `STOP`/`STATUS` 流程验证闩锁。

8. 可选 **编码器符号反**：交换某一路编码器 A/B 后运行较高目标（≥50% 占空比、≥3% Vmax 速度）。预期 300 ms 内 `FAULT encoder_sign`。若重新标定，标定会自动吸收符号，不应再报此故障。

9. **面板故障提示 UI（独立于脚本故障测试）**：重新连接面板、应用布局、勾选“场地已清空，车轮/电机已固定，可开始本次测试”（重连/STOP 会清除）并勾选闭环，启动一次 `SPEED`，拔编码器触发故障。确认状态变为“故障闭锁：...”、运动按钮禁用；此时**不要断开面板**，直接按“停止全部 / Esc”发送显式 `STOP` 清除闩锁。记录后重新连接/应用布局并重新勾选现场确认。面板不能替代脚本的固件拒绝验证。

记录：

| 测试 | 触发方式 | 实际 FAULT | 触发到停车时间 | 闭锁拒绝 | STOP 后恢复 |
|---|---|---|---|---|---|
| 编码器断开 30%（脚本） | | | | | |
| 编码器断开 5%（脚本） | | | | | |
| 编码器断开 10%（脚本） | | | | | |
| 堵转（脚本） | | | | | |
| 符号反（可选，脚本） | | | | | |
| 模式切换清除闩锁 | | | | | |
| 链路超时撤销控制权清除闩锁 | | | | | |
| 面板故障提示 | | | | （UI 禁用） | |

判定：故障后不得持续输出；`SPEED`/`SPEEDW`/`CAL` 被拒绝；显式 `STOP` 后 `FAULT none`；全零 `SPEED` 不解除固件闩锁；5%/10% 低速断编码器也必须触发 `no_feedback`。故障闩锁也会在切换模式或撤销控制权（断链/超时）时清除，且必须按 6b 实际验证；这条不改变“故障后必须显式 STOP 才能继续”的日常操作要求。

**用户结果（待填）**：通过 / 不通过；备注：

---

## 步骤 7：通信超时与断电/急停（分开测）

**工具：串口脚本（7A 链路静默）；面板/蓝牙（7B 可选）；断电测试单独执行（7C）。**
**目的**：确认 1.5 s 通信超时在 ESP32 持续供电时确实停车；断电行为单独记录，不作为超时通过依据。

### 7A. 链路静默（固件超时，ESP32 保持供电）

1. 四轮架空或落地安全条件；确认标定有效、布局可应用。运行：

```powershell
& 'D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe' "$env:TEMP\chassis_raw.py" --port COM4 --layout CBAD --link-timeout
```

2. 脚本会发送 `MODE CHASSIS`、`LAYOUT CBAD 1 1 1 1`、`SPEED 50 0 0 5000`，**先读到 `SPEED_RUN`**，并保持心跳 0.5 s 后才暂停心跳 3 s；期间**继续读串口**，且所有写串口操作由同一把锁串行化。ESP32 仍由 USB 供电。
3. 暂停前必须**同时**满足：脚本打印 `SPEED_RUN`，并且肉眼确认四轮确实在转。只有 `SPEED_RUN` 行不算运行证据（可能是回显/陈旧行）；只有轮子转也不算，因为可能是开环或其他端口。
4. 脚本在暂停心跳后打印 `TEST WINDOW START`，只有此标记之后收到的 `STOPPED link_timeout` 才算通过；脚本同时打印从暂停到该行的 elapsed 时间。必须看到：

```text
TEST WINDOW START
STOPPED link_timeout
[elapsed x.xx s after heartbeat pause]
```

5. 观察四轮在约 1.5 s 内停转；用脚本打印的 elapsed 记录，而不是只看“轮子停了”。脚本退出时会发送 `STOP`。
6. 若改用“强制结束面板进程”的方式，ESP32 仍由 USB 供电，但日志会丢失；只能物理确认停车，不能替代 7A 对 `STOPPED link_timeout` 的捕获。
7. 重新连接面板/脚本，重新 `MODE CHASSIS` 并应用布局；`STATUS` 应为 `STOPPED ALL`、`FAULT none`；`CAL?` 仍为 `VALID=1`。布局不会自动恢复。

判定：必须在 `TEST WINDOW START` 之后看到 `STOPPED link_timeout`，且暂停前已确认 `SPEED_RUN` + 轮子实际转动；仅“轮子停了”不能证明是固件超时（可能是断电/接触不良）。

### 7B. 蓝牙链路丢失（可选，若使用蓝牙）

1. 面板通过蓝牙连接，应用布局并勾选“场地已清空，车轮/电机已固定，可开始本次测试”（重连/STOP 会清除），启动一次定时运动（闭环 `SPEED` 或开环 `MECANUM`），保持 ESP32 USB 供电。
2. 关闭蓝牙适配器/移出范围，观察四轮停车；记录时间。
3. 恢复蓝牙，重新连接，重新 `MODE CHASSIS`、应用布局并**重新勾选现场确认**，`STATUS` 应为 `STOPPED ALL`、`FAULT none`。
4. 蓝牙断开时串口日志可能丢失，`STOPPED link_timeout` 行可能无法捕获；本项只记录“观察到的停车时间”，不把未捕获的行当作已确认。若需要精确行，使用 7A 的 USB 链路静默方式（同一固件超时逻辑）。

### 7C. 断电/急停（不是 1.5 s 超时测试）

1. 仅在四轮架空或车辆可靠约束时进行。
2. **切断驱动电池（ESP32 仍由 USB 供电）**：记录现象。预期固件继续运行，PWM/STBY 仍有定义，但电机失去转矩；这检验动力断电，不是 1.5 s 超时。
3. **拔掉 ESP32 USB（逻辑断电）**：记录现象。预期固件停止运行，PWM/STBY 不再受控，电机输出可能滑行、停止或出现未定义状态。
4. 以上两项都**不得**声称是 1.5 s 超时停车，也不得把结果计入 7A 的通过条件。实体急停应优先切断驱动电源，这是最后的安全手段。

判定：

- 7A：ESP32 供电、链路静默时 ≤1.5 s 停车，并捕获 `STOPPED link_timeout`。
- 7B（如做）：蓝牙丢失后停车；重连不自动恢复控制。
- 7C：只记录断电行为；不作为超时验收。

**用户结果（待填）**：7A 通过/不通过；7B 通过/不通过/未做；7C 已记录/未做；备注：

---

## 步骤 8：记录电池、起转死区、稳态误差与打滑

**工具：面板记录 + 万用表/卷尺。**
**目的**：形成可复现的现场基线，供后续调参和对比。

记录：

| 项目 | 数值 / 说明 |
|---|---|
| 电池电压（静止 / 运行中） | |
| 环境温度 / 地面材质 | |
| 四轮起转死区（`CALW DEADBAND=a`，%） | A: B: C: D: |
| 实测最低起转占空比（如做手动验证） | A: B: C: D: |
| 30% 目标稳态误差（步骤 3） | A: B: C: D: |
| 60% 目标稳态误差（步骤 3） | A: B: C: D: |
| 30% 目标稳态占空比 | A: B: C: D: |
| 直行打滑轮位 / 现象（步骤 4） | |
| 横移/旋转打滑轮位 / 现象（步骤 5） | |
| 标定 `VMAX` / `LIMIT` / `min(vsim)` | |
| 是否重新标定 / 原因 | |

补充观察（自由记录）：起转是否抖动、停车是否滑行、蓝牙下遥测是否丢行、`ENC invalid` 是否增长、电池压降是否明显。

**用户结果（待填）**：记录完成 / 未完成；备注：

---

## 总体验收判定

只有以下全部满足，才可把本轮闭环标记为“实机验收通过”：

- [ ] 步骤 1 标定成功，`VALID=1`，四轮符号/V100/a/b 合理，无 `CAL FAIL`。
- [ ] 步骤 2 重启后 `CAL?` 仍为 `VALID=1`，参数一致。
- [ ] 步骤 3 悬空低速/中速跟随稳定，无故障；稳态误差在用户认可范围内（建议 ±5%）。
- [ ] 步骤 4 落地直行闭环不差于开环，四轮计数差减小；打滑残余偏航已记录并接受。
- [ ] 步骤 5 横移/斜移/旋转方向正确，闭环不差于开环。
- [ ] 步骤 6 编码器断开（含 5%/10% 低速）/堵转能触发 `FAULT` 并闭锁；显式 `STOP` 后恢复；全零 `SPEED` 不解除闩锁；6b 的切换模式/撤销控制权清除闩锁已实际确认。
- [ ] 步骤 7A ESP32 持续供电、链路静默时 ≤1.5 s 停车，并捕获 `STOPPED link_timeout`；7C 断电测试单独记录，未冒充超时通过。
- [ ] 步骤 8 已记录电池电压、死区、稳态误差和打滑情况。

已知限制（不作为本轮失败判据，但必须向使用者说明）：

- 轮速闭环控制的是轮子转速，不是车体速度；打滑仍会造成残余偏航，不能保证直线或航向。
- 标定在悬空状态完成，落地带载的最大速度低于标定值；`LIMIT=80` 提供调节余量，四轮同时饱和时只能降速。
- 软件编码器中断在高速/蓝牙开启时可能丢边沿；需要靠 `invalid` 与速度一致性观察，后续可迁移 PCNT。
- 没有物理单位、IMU、视觉、里程计或电流环；不承诺 mm/s、位置、旋转角度、绝对直线度或四轮绝对同步。
- `0 < |target| < 5%` 的非零闭环轮目标不支持；某个轮目标为 0（例如纯横移/旋转混控中）是允许的。
- v1 的 `CAL?` 不暴露逐轮 `vsim`；只能得到 `VMAX/LIMIT` 推导的 `min(vsim)` 和 `CALW` 的每轮 `V100`。

验收人签字 / 日期：____________________

> 在签字前，本文件中的实机结果均视为“待用户执行”；离线测试通过只代表代码与构建回归通过。
