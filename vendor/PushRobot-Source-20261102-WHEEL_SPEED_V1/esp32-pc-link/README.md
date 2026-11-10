# esp32-pc-link：电脑 ↔ ESP32 的 DRY_RUN 通信与调试面板

> 2026-10-20 更新：日常使用根目录 `run_debug.cmd`；当前开发板统一烧录 `motor-bringup` 的 FOUR_V1，支持模拟与 A/B/C/D 四路逐路台架，无需切换烧录。本目录保留原纯模拟固件和共用协议核心；下文“不配置电机 GPIO”仅描述本目录原 `src/main.cpp`，统一固件会初始化全部电机引脚并默认关闭输出。新说明见 `debug-workbench/README.md`。

本工程是独立新增的，**没有修改**已通过的 `esp32-connection-test/`。

当前阶段只做：

- 电脑与 ESP32 的 USB 串口通信；
- DRY_RUN 固件：接收并回传运动目标，**不配置任何电机 GPIO，不碰 STBY**；
- Tkinter 调试面板；
- 独立的摄像头预览（与运动发送完全分开）。

尚未完成：电机/D24A、供电、编码器、视觉标定与推块闭环。

---

## 1. 目录

```text
esp32-pc-link/
  platformio.ini
  src/main.cpp                  Arduino 入口：只用 Serial + LinkCore
  lib/link_core/                纯 C++ 协议状态机（可在电脑上测试）
  host-tests/                   g++ 离线测试
  pc/
    setup.ps1 / setup.cmd       创建 .venv，安装 pyserial/opencv-python
    requirements.txt
    link_client.py              串口线程、分包/粘包、发送队列
    panel.py                    DRY_RUN 调试面板（逻辑）
    panel_view.py               面板布局与样式（左侧控制/右侧目标与日志）
    camera_source.py            可复用的单摄像头取图接口（给视觉模块）
    camera_preview.py           独立摄像头预览（含模拟画面）
    link_smoke_test.py          对实物 ESP32 的验收脚本
    tests/                      假串口离线测试
    run_camera.cmd              独立摄像头预览入口；面板统一使用根目录 run_debug.cmd
    run_smoke_test.ps1 / .cmd
    run_tests.ps1 / .cmd
  README.md
  TEST-REPORT.md
```

---

## 2. 快速开始

### 2.1 电脑端环境

在 `pc/` 下运行：

```powershell
.\setup.cmd
```

它会用系统 Python 3.14 创建 `.venv`，安装 `pyserial` 和 `opencv-python`。系统 Python 必须带 tkinter；脚本会先检查。

在当前 `pc/` 目录启动统一面板（进入后可切换模式）：

```powershell
..\..\run_debug.cmd
```

使用 `.cmd` 启动即可，无需修改系统执行策略。若需要单独运行原 DRY_RUN 面板，可执行：

```powershell
.\.venv\Scripts\python.exe panel.py
```

### 2.2 编译与烧录 ESP32

在 `esp32-pc-link/` 下：

```powershell
pio run
pio device list
pio run -t upload --upload-port COM4
```

`platformio.ini` **没有写死** COM4。每次先确认设备管理器/`pio device list` 中 CP2102 的实际端口；端口可能变化。上传时若提示连接失败，按 BOOT、开始写入后松开，结束后按 EN。

本固件是 `DRY_RUN`，烧录不会让电机转动。不要在本阶段连接电机电源，也不要在 D24A 无电时把控制信号接上去。

### 2.3 面板操作

1. 点“刷新”，选择 CP2102 对应的端口，点“连接设备”。
2. 面板先等待 ESP32 启动并确认 `hello`/协议；确认后自动发送 `STOP`，再发 `STATUS`。
3. 只有设备进入 `DISARMED` 且模式为 `DRY_RUN` 时，“模拟使能”才会启用。
4. 点“模拟使能”；按住左侧方向键，松开后自动发零速；红色“停止并撤销使能”按钮（或 Esc）撤销使能。
5. 左侧是控制区；右侧上方是设备回传的 vx/vy/ω，右侧下方是“通信日志 / 设备详情 / 协议工具”。
6. 顶部三张卡片显示连接状态、控制状态、最近有效应答；连接异常或 ARMED 会用颜色突出。
7. 默认日志只保留关键事件，周期 STATUS/DRIVE 应答不会刷屏；“详细收发”可看全部原始行。
8. 底层参数（固件、会话、序号、启动编号等）收在“设备详情”页；PING/STATUS 和原始命令在“协议工具”页。
9. 窗口失去焦点会自动 STOP 并撤销使能。
10. 右侧“摄像头”页可以预览手机画面：点“打开摄像头”后使用 `camera_config.json` 的源/后端；只预览，不发送串口或运动指令；面板关闭时会释放摄像头。

面板不会在打开时自动连接，也不会自动 ARM；断开/重连后必须重新 ARM。摄像头页也不会自动打开，必须手动点“打开摄像头”。

### 2.4 摄像头预览

摄像头与运动发送是分开的。打开预览不会控制机器人。

**本机设备名确认：编号 0 = Camo，编号 1 = USB2.0 HD UVC WebCam（笔记本摄像头）。**  
Camo 使用 Media Foundation（msmf）、编号 **0**。`pc/camera_config.json` 已设为默认；现在直接运行：

```powershell
.\run_camera.cmd
```

等价显式命令：

```powershell
.\run_camera.cmd --source 0 --backend msmf
```

其他命令：

```powershell
.\run_camera.cmd --names
.\run_camera.cmd --list
.\run_camera.cmd --source 0 --backend msmf
.\run_camera.cmd --source "rtsp://..."
.\run_camera.cmd --synthetic
.\run_camera.cmd --synthetic --headless --frames 30
```

- `.\run_camera.cmd --names` 用 DirectShow 列出设备名：本机 **0 = Camo，1 = USB2.0 HD UVC WebCam**。编号顺序通常在 MSMF 下相同，但不保证；最终以画面为准。
- 之前把编号 1 当成 Camo 是错的：编号 1 实际是笔记本摄像头。现在配置已改为 **编号 0 + msmf**。

后端含义：

| 值 | 含义 | 本机建议 |
|---|---|---|
| `msmf` | Windows Media Foundation，现代相机 API；对 Camo/UVC 支持较好 | **Camo 使用这个** |
| `dshow` | Windows DirectShow，旧 API；兼容设备多，但某些虚拟摄像头会阻塞/取流慢 | 本机 Camo 能打开但长跑读取阻塞，不用 |
| `auto` | `CameraSource` 先试 DirectShow，失败再用 OpenCV 默认后端；不是“自动挑最快” | 本机不要用 |
| `默认后端`（仅 `--list`） | 不指定后端，让 OpenCV 自己选择（可能是 MSMF 或 DirectShow） | 仅用于探测 |

不同后端的设备编号可能不同；编号相同也不保证是同一台物理设备，必须用 `--names` 和画面确认。
- **Camo 免费版上限是 720p/30**；1080p/4K 和更高帧率属于 Camo Pro。因此“16:9 只有 720p”不是 OpenCV 或面板的问题，是当前 Camo 版本限制。
- 当前 Camo 实际输出 **1280x720**，OpenCV 连续读取约 22–24 FPS（摄像头上报 30，实际传输/处理略低）。`--width/--height` 只是请求，最终以“实际分辨率”输出为准。
- 面板内“摄像头”页的预览会缩放到 480x270 以适配窗口；**缩放只影响预览，不改变实际采集分辨率**。状态行会显示实际的 `1280x720` 和 FPS。
- `--list` 会同时探测 MSMF/DirectShow/默认后端；同一编号在不同后端可能对应不同设备，必须用画面确认。
- 运行后按 `q`/ESC 退出，按 `s` 保存截图；`--headless --frames N` 会在结束时打印实际分辨率。
- `--synthetic` 是模拟画面，不是摄像头；用于离线验证显示链路。
- 当前工程没有做人脸/目标识别、场地标定或 ArUco；这些由队友的视觉模块后续接入。
- 给队友的取图接口是 `pc/camera_source.py` 的 `CameraSource`：`open()` / `read()` 返回 BGR 帧 / `close()`，不连接串口、不控制机器人。本机使用 `CameraSource(0, backend="msmf")`。视觉结果应由控制层生成 DRIVE，而不是在取图循环里直接发串口命令。

#### iPhone + Camo 实际步骤

1. iPhone 安装 **Camo Camera**。
2. Windows 安装 **Camo Studio**，并在引导中安装 **compatibility add-on**（需要管理员权限；否则 Windows 其他程序看不到 `Camo Camera`）。
3. 用 USB 连接 iPhone，在 Camo Studio 中看到画面。
4. 先在 Windows“相机”应用中确认 `Camo Camera` 画面正常。
5. 运行 `.\run_camera.cmd --names` 和 `.\run_camera.cmd --list`，再运行 `.\run_camera.cmd --source 0 --backend msmf` 目视确认。
6. 本机已确认 `0 + msmf` 可读帧且编号 0 名称为 Camo；如果 USB 拓扑或 Camo 变化，重新执行第 5 步并更新 `camera_config.json`。

摄像头取图已打通；**固定俯视安装、1080p 选择、场地标定和 ArUco 尚未完成**，不能把当前结果当成视觉闭环。

---

## 3. 通信协议 v1

- 串口：115200，8-N-1。
- 电脑 → ESP32：逐行 ASCII，`\n` 结尾，最长 120 字符（不含 CR/LF）。
- ESP32 → 电脑：每行一个 JSON，全部带 `"v":1`。
- 方向约定：`x` 向前、`y` 向左、`w` 逆时针为正。
- 速度范围：`vx/vy` 为 `-300..300` mm/s；`w` 为 `-180..180` deg/s。

### 3.1 命令

| 命令 | 说明 |
|---|---|
| `PING` | 连通性检查，返回 `pong` |
| `STATUS` | 查询状态；**不刷新**看门狗 |
| `ARM <session> <arm_token>` | 进入 ARMED；必须带当前 token；session 为非零 31 位随机数 |
| `DRIVE <session> <seq> <vx> <vy> <w>` | 仅在 ARMED 后接受；session 必须匹配，seq 必须严格递增 |
| `STOP` | 立即清目标并撤销使能；始终接受 |
| `DISARM` | 同 STOP，原因记为 DISARM；始终接受 |

`STOP`/`DISARM` 即使带多余参数也会执行，以避免因格式错误而无法停止。

### 3.2 状态机规则

- 上电默认 `DISARMED`。
- ARM 成功时启动 300ms 计时；之后必须有**有效的新 DRIVE** 才能维持。
- 零速度 `DRIVE` 也算有效刷新；`PING`、`STATUS`、非法值、重复 seq、错误 session 都不刷新。
- 超时判断用无符号差值：`elapsed = now - last_drive_ms; elapsed >= 300`，可处理 `millis()` 回绕。`feed()` 处理字节前和 `tick()` 都会先更新看门狗。
- 超时后清目标、进入 `DISARMED`、发出 `WATCHDOG_STOP` 事件；必须重新 ARM。
- 每次 ARM 使用新的 session；同一次启动内，最近一次成功用过的 session 在断开使能后不能再次 ARM（`SESSION_REUSED`）。
- ESP32 启动生成随机 `boot_id` 和 `arm_token`；`hello`/`status` 都会返回。
- `arm_token` 在每次 `STOP`、`DISARM`、看门狗超时后重新生成。旧 token 的 ARM 会被拒绝（`STALE_ARM_TOKEN`）。
- 因此：STOP/DISARM/超时后、以及 ESP32 重启后，旧 ARM 报文不能重放；旧 DRIVE 在非 ARMED 状态下返回 `NOT_ARMED`。防的是过期/重复报文，不是密码学身份认证；目前不写 NVS。
- `boot_id` 变化时，面板清空 session、停止 DRIVE，并要求重新同步/ARM；不会自动 ARM。

### 3.3 典型 JSON

`hello`：

```json
{"v":1,"type":"hello","fw":"0.1.0","mode":"DRY_RUN","protocol":1,
 "boot_id":123456,"arm_token":654321,"watchdog_ms":300,
 "state":"DISARMED","uptime_ms":1234}
```

`status` 主要字段：

```json
{"v":1,"type":"status","state":"ARMED","session":111,"last_session":111,
 "last_seq":7,"vx":100,"vy":0,"w":0,"age_ms":40,"watchdog_ms":300,
 "reason":"NONE","boot_id":123456,"arm_token":654321,
 "fw":"0.1.0","mode":"DRY_RUN","protocol":1,"dry_run":true}
```

`ack` 包含 `cmd`、`ok`、`error`、`state`、`session`、`last_seq`、`vx/vy/w`、`reason` 等字段；`ok=false` 时看 `error`。

错误码：

`BAD_LINE`、`TOO_LONG`、`BAD_ARGS`、`UNKNOWN_CMD`、`NOT_ARMED`、`ALREADY_ARMED`、`STALE_ARM_TOKEN`、`SESSION_REUSED`、`SESSION_MISMATCH`、`STALE_SEQ`、`RANGE`。

### 3.4 分包/粘包/慢速输入

固件按字节累积，遇到 `\n` 才处理一行；不依赖 `readStringUntil` 的 50ms 超时，因此支持：

- 任意分包/粘包；
- 人工逐字节慢速输入；
- CRLF；
- 超长行丢弃并回 `TOO_LONG`，之后在下一个换行重新同步；
- 控制字符/非 ASCII 回 `BAD_LINE`。

---

## 4. 电脑端安全行为

- **不堆积 DRIVE**：发送端只保留最新运动目标；界面 10Hz 更新，写线程合并；未发送的旧目标不消耗 seq。
- **STOP 最高优先级**：清空待发 DRIVE 和待执行控制命令，然后立即发送 STOP。已经写到串口线上的那一帧无法撤回，但 STOP 紧随其后。
- **界面卡顿保护**：10Hz 定时器发现间隔超过 350ms 时，不补发旧动作，主动发 STOP 并要求重新 ARM。
- **连接时序**：打开端口后先等 `hello`（约 1.5s）；如果板子没有因开串口复位，则用 PING 探测；在收到合法 v1 响应前不发 STOP/STATUS/ARM。确认后发 STOP → STATUS，只有 DISARMED 才进入 READY。
- **断线/重连**：串口读/写异常会清空 session 和待发运动，界面停止定时器；不会自动重连、不会自动 ARM、不会重发缓存动作。异常断开时还会清空固件、模式、状态、会话、目标、“最近有效应答”等显示，不残留上一条连接的值。
- **迟到应答**：
  - STOP/DISARM 之后迟到的 ARM 应答按本地控制代次丢弃，不会重新建立 session；
  - 旧 session 的 DRIVE 应答丢弃，不会改新 session 的序号/目标；
  - STOP/超时后短窗口内迟到的 ARMED 状态忽略；
  - 重新 ARM 后短窗口内迟到的 DISARMED 状态忽略；
  - `can_arm` 在 ARM 请求未确认或 STOP 尚未确认时保持禁用。
- **日志噪声**：周期 STATUS/PONG/DRIVE 应答默认不进日志；“最近有效应答”卡片单独显示最近一条合法报文，关键事件（ARM/STOP/看门狗/重启/错误）仍写日志。
- **每次 ARM 新 session**：面板用 `secrets` 生成 31 位随机非零数。

---

## 5. 测试

### 5.1 离线/模拟

```powershell
# C++ 协议核心
cd host-tests
.\run_tests.cmd

# Python 串口客户端（假串口）
cd ..\pc
.\run_tests.cmd

# 面板只构建界面，不连接
..\..\run_debug.cmd --self-test

# 摄像头模拟画面
.\run_camera.cmd --synthetic --headless --frames 30
```

### 5.2 实物 ESP32

确认串口空闲后：

```powershell
cd pc
.\run_smoke_test.cmd --port COM4
```

脚本会测试：身份/协议、慢速逐字节 PING、超长行恢复、未 ARM 的 DRIVE、ARM+DRIVE、重复 seq、零速刷新与看门狗、STOP、旧 token/session 重放、端口保持打开但停止发命令（模拟 PC 卡死）、子进程带使能异常退出后重连、重启后不恢复动作。

其中两项需要人工操作：

- 拔下/重新插上 ESP32 的 USB；
- 按一下 ESP32 的 EN 复位键。

不接受人工操作时用 `-SkipManual`，脚本会把这两项标成 `SKIP`，不会算作通过。

面板自身也可以做一次真实联调（ARM → 方向映射 → 零速 → STOP）：

```powershell
.\.venv\Scripts\python.exe panel.py --live-test --port COM4
```

### 5.3 本次实测结果

见 `TEST-REPORT.md` 和 `pc/smoke_results.json`。要点：

- 主机协议测试 126 项通过；
- PC 端离线测试 19 项通过（14 项串口客户端/帧解析 + 5 项面板状态）；
- 实物自动验收 20 项通过、0 失败、2 项人工跳过；
- 300ms 看门狗多次实测约 322–350ms（轮询分辨率约 20ms，精确边界由主机测试覆盖）；
- 真实面板联调 PASS（ARM、方向 vx=100、松开零速、STOP）；
- 摄像头取图已打通：设备名确认 **0 = Camo**，`--source 0 --backend msmf` 读取到 **1280x720**，连续读取约 25–28 FPS；面板“摄像头”页自检通过。固定俯视安装、1080p（Camo Pro 或换 USB 摄像头）和标定尚未完成。

---

## 6. 常见问题

| 现象 | 检查 |
|---|---|
| PowerShell 提示“禁止运行脚本” | 用 `..\..\run_debug.cmd` 等 `.cmd` 包装器，或直接运行 `.\.venv\Scripts\python.exe panel.py`；不必修改系统执行策略 |
| 找不到 COM 口 | USB 数据线、CP2102 驱动、设备管理器；重新插拔 |
| 打开 COM4 失败 | 关闭 VS Code Monitor、其他串口工具；不要强行结束用户进程 |
| 上传停在 Connecting | 按住 BOOT 开始上传，出现写入后松开；结束按 EN |
| 连接后一直“等待启动” | 按 EN 复位；确认波特率 115200；确认没有别的程序占用串口 |
| 面板显示协议错误 | 固件版本/协议不匹配；确认烧的是本工程 DRY_RUN 固件 |
| ARM 失败 `STALE_ARM_TOKEN` | 设备状态已变化（超时/STOP/重启）；刷新状态后重新点 ARM |
| 看门狗超时 | 松开方向键或界面卡顿会停止；重新 ARM |
| 摄像头 `--list` 没有结果 | 先确认 Camo Studio 和 Windows 相机应用有画面；本机 Camo 使用 `--source 0 --backend msmf` |
| 打开的是笔记本摄像头 | 本机编号 1 是 USB2.0 HD UVC WebCam；Camo 是编号 0。运行 `--names` 确认，再把 `camera_config.json` 改成 `source: 0` |
| 启动前乱码 | 现有基线和本固件都可能出现；面板忽略非 JSON 行，**未声称已根治** |

---

## 7. 已知边界

- 本阶段没有电机、D24A、6V 电源测试；固件没有电机 GPIO 代码。
- USB 拔掉会同时给 ESP32 断电，因此“链路丢失但 ESP32 仍带电”的看门狗场景，当前用“端口保持打开、停止发命令”模拟；以后用电池给 ESP32 供电后需要再实测一次。
- 摄像头取图路径已确认：本机 Camo 走 MSMF 编号 0，当前 1280x720，OpenCV 连续读取约 25–28 FPS；Camo 免费版上限就是 720p/30。需要 1080p 时使用 Camo Pro，或换一个固定 USB 摄像头（长期俯视通常更省心）。固定安装和标定尚未完成。`--list` 探测到的编号不代表设备身份，仍要用 `--names` 和画面确认。
- 视觉识别/模型训练由队友负责；本工程只提供独立取图/预览和清晰的通信接口，尚未接入视觉闭环。
- 固定俯视标定、ArUco、机器人位姿识别尚未实现。
- 启动乱码原因未确定；面板能容忍，但不等于已修复。
