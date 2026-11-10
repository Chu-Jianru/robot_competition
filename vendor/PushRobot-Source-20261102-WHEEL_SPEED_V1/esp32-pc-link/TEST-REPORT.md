# DRY_RUN 通信与调试面板：测试报告

日期：2026-10-19  
工程：`esp32-pc-link/`（新增工程，未修改 `esp32-connection-test/`）

## 1. 环境与对象

| 项目 | 实际值 |
|---|---|
| 操作系统 | Windows 11 Home China 10.0.26200 |
| 系统 Python | 3.14.5（带 tkinter） |
| 虚拟环境 | `pc/.venv` |
| pyserial | 3.5 |
| OpenCV | 5.0.0.93 |
| numpy | 2.5.4 |
| PlatformIO Core | 6.2.0 |
| 开发板 | ESP32-WROOM-32 / CP2102，COM4 |
| 固件 | `esp32-pc-link 0.1.0`，协议 v1，`DRY_RUN` |

固件构建与烧录均成功。烧录后设备能正常启动并响应协议。

## 2. 离线 / 模拟测试（不是实物电机测试）

### 2.1 C++ 协议核心

命令：

```powershell
cd host-tests
.\run_tests.ps1
```

结果：**126 项通过，0 失败**。

覆盖：

- 启动 hello / 字段；
- 分包、粘包、逐字节慢速输入、CRLF、空行；
- 超长行 `TOO_LONG` 与换行后重新同步；
- 控制字符 `BAD_LINE`；
- 非法数值、溢出、错误参数个数；
- 未 ARM 的 DRIVE；
- `ARM <session> <token>`、重复 ARM、旧 token、session 复用；
- 有效 DRIVE、重复/旧 seq、错误 session、越界值；
- 零速度 DRIVE 刷新看门狗；
- PING/STATUS/非法命令不刷新看门狗；
- 300ms 边界：299ms 仍 ARMED、300ms DISARMED；边界时刻到达的 DRIVE 被拒；
- `millis()` 回绕前后 299/300ms；
- STOP/DISARM、旧 token/session、重启后旧 ARM/旧 DRIVE；
- 状态 JSON 字段与方向/单位。

### 2.2 PC 端离线测试（假串口 + 面板状态）

命令：

```powershell
cd pc
.\run_tests.cmd
```

结果：**19 项通过，0 失败**（14 项串口客户端/帧解析 + 5 项面板状态）。

覆盖：

- 行缓冲分包/粘包/超长行；
- 身份确认前不发送 STOP/STATUS；
- 收到 hello 后按顺序 STOP → STATUS，只有 DISARMED 才 READY；
- 连接时设备已 ARMED 会被 STOP；
- ARM → DRIVE → STOP；
- 大量 DRIVE 合并、STOP 优先、STOP 后无 DRIVE；
- 断线清空状态、重连不自动运动；
- `boot_id` 变化触发重新同步、不自动 ARM；
- ARM token 失效不自动重试；
- **STOP 后迟到的 ARM 应答被忽略**；
- **STOP 后迟到的 ARMED 状态被忽略**；
- **重新 ARM 后迟到的 DISARMED 状态被忽略**；
- **重新 ARM 后旧 session 的 DRIVE 应答被忽略**；
- 面板异常断开后清空连接/固件/状态/会话/目标/应答显示，不残留旧值；
- 面板重启事件清空会话/目标显示；
- 周期 STATUS 默认不写入日志；“最近有效应答”单独更新；
- ARM 失败等关键错误仍然写入日志并显示在底栏；
- 新面板包含“摄像头”页，但不会自动启动摄像头。

### 2.3 面板与摄像头离线检查

| 检查 | 结果 |
|---|---|
| `panel.py --self-test` | 通过（构建新布局：左侧控制、右侧目标与日志，不连接串口） |
| `camera_preview.py --synthetic --headless --frames 30` | 通过，处理 30 帧，约 257 FPS（模拟画面，仅验证显示链路） |
| `panel.py --camera-self-test` | 通过：面板“摄像头”页读取 Camo 6 帧，状态显示 `源 0 | 1280x720`；3 秒运行约 85 帧（约 28 FPS） |

摄像头实物状态（2026-10-19 用户实测后更新）：

- 用户已安装 Camo Camera（iPhone）+ Camo Studio（Windows）。
- **设备名枚举（pygrabber / DirectShow）确认：编号 0 = Camo，编号 1 = USB2.0 HD UVC WebCam（笔记本摄像头）。** 之前把编号 1 当成 Camo 是误判：编号 1 实际是电脑摄像头。
- 修正后使用 `CameraSource(0, width=1920, height=1080, backend="msmf")` 实测：打开成功，实际分辨率 **1280x720**；2 秒读取约 **28 FPS**。
- 对比：`source=0, backend=dshow` 也能打开 720p，但 2 秒只读到 2 帧（读取阻塞），不作为可用路径；MSMF 稳定。
- Camo 官方定价页确认：**免费版上限 720p/30**，1080p/4K 和更高帧率属于 Camo Pro。因此 16:9 只有 720p 是 Camo 免费版限制，不是 OpenCV 或面板的问题。
- 项目已写入 `pc/camera_config.json`，默认 `source=0, backend=msmf`；`.\run_camera.cmd` 无需参数即可打开 Camo。新增 `--names` 用于列出 DirectShow 设备名。
- 面板右侧新增“摄像头”页：手动“打开摄像头”后显示 480x270 预览（缩放不影响实际采集），状态行显示实际分辨率和 FPS；只读，不发送串口/运动指令。
- 面板摄像头页 3 秒运行 **85 帧（约 28 FPS）**，状态显示 `源 0 | 1280x720`。
- **未完成**：固定俯视安装、Camo 1080p 配置、场地标定、ArUco/位姿识别。当前结果只证明 OpenCV 取图路径打通，不代表视觉闭环完成。

## 3. ESP32 实物测试

### 3.1 自动验收

命令：

```powershell
cd pc
.\.venv\Scripts\python.exe link_smoke_test.py --port COM4 --skip-manual
```

最终结果：**20 项通过，0 失败，2 项人工步骤跳过**。  
结果文件：`pc/smoke_results.json`。

关键实测：

| 项目 | 结果 |
|---|---|
| 身份 / 协议 | `fw=0.1.0`、`mode=DRY_RUN`、协议 v1，PASS |
| 同步 DISARMED | `reason=STOP`，PASS |
| 逐字节慢速 PING（40ms/字节） | 收到 PONG，PASS |
| 300 字符超长行 | 返回 `TOO_LONG`，随后 PING 恢复，PASS |
| 未 ARM 的 DRIVE | `NOT_ARMED`，PASS |
| ARM + DRIVE | session 匹配；目标 `50,-20,10`，`last_seq=1`，PASS |
| 重复 seq | `STALE_SEQ`，目标保持 `50,-20,10`，PASS |
| 零速 DRIVE 刷新 + 看门狗 | 多次实测约 **322–350ms** 后 DISARMED/WATCHDOG（轮询分辨率约 20ms；精确 299/300ms 边界由主机测试覆盖），PASS |
| 看门狗后旧 token | `STALE_ARM_TOKEN`，PASS |
| 看门狗后旧 session | `SESSION_REUSED`，PASS |
| STOP | `state=DISARMED`、`reason=STOP`，PASS |
| STOP 后旧 session DRIVE | `NOT_ARMED`，PASS |
| STOP 后旧 token ARM | `STALE_ARM_TOKEN`，PASS |
| 端口保持打开、停止发命令 500ms | DISARMED/WATCHDOG，PASS |
| 子进程 ARM+DRIVE 后 `os._exit` | 重连后 DISARMED，boot_id 变化；旧 session DRIVE 被拒，旧 token ARM 被拒，PASS |
| USB 拔插（人工） | **SKIP（未执行）** |
| EN 复位（人工） | **SKIP（未执行）** |

说明：第一次运行自动验收时，“逐字节慢速 PING”失败，原因是测试脚本自己的 `send_slow()` 没有补换行符，不是固件问题；修正测试脚本后复跑通过。

### 3.2 真实面板联调

命令：

```powershell
cd pc
.\.venv\Scripts\python.exe panel.py --live-test --port COM4
```

结果：**PASS**

实际执行了：

- 使用新布局构建面板：左侧控制区，右侧目标值 + 日志/详情/工具页；连接状态、控制状态和红色 STOP 醒目；底层参数收在“设备详情”；
- 面板连接 COM4，等待身份，自动 STOP/STATUS，进入 READY；
- 点 ARM（程序调用面板 ARM 流程）；
- 方向映射 `forward`，确认 `vx=100`；
- 松开方向，确认 `vx=0`；
- STOP，确认设备 DISARMED，并确认 STOP 后没有重新建立会话；
- 全程 DRY_RUN，没有电机电源，没有 GPIO 动作。

输出：

```text
panel live test PASS: armed=True direction_vx=100:True release_zero=True stopped=True
```

（session 为随机值，每次运行不同。）

## 4. 安全边界核对

- 固件中没有 `pinMode`、`digitalWrite`、`ledc`、STBY、电机或 D24A 相关代码；源码中只有说明性注释。
- 没有连接电机、D24A 或 6V 电源；没有操作任何电机 GPIO。
- `esp32-connection-test/` 的文件时间仍为 2026-10-13，未被本次修改。
- 摄像头预览与运动控制代码完全分离；打开预览不会发送串口命令。

## 5. 尚未完成 / 需要用户后续操作

1. **人工 USB 拔插测试**：将 ESP32 的 USB 拔下 2 秒再插上，然后重新连接，确认 DISARMED。  
   可运行 `.\run_smoke_test.cmd --port COM4` 并在提示时操作；不操作时加 `--skip-manual`。
2. **人工 EN 复位测试**：连接/使能后按一下 EN，确认面板识别新 `boot_id` 并要求重新 ARM。  
   自动重连测试已经证明重置后不会恢复旧动作，但“面板在线时检测到 EN 复位”的人工步骤尚未执行。
3. **摄像头固定安装与标定**：Camo 取图已打通（本机为 MSMF 编号 0，1280x720，约 28 FPS；面板“摄像头”页可预览）；待做：固定俯视安装、场地标定、ArUco/位姿识别。免费版 Camo 上限 720p/30；需要 1080p 时选 Camo Pro 或换固定 USB 摄像头。当前不是视觉闭环。
4. **电机 / D24A / 供电 / 编码器**：本阶段完全没有测试。
5. **视觉识别、场地标定、ArUco、推块闭环**：未实现；由队友的视觉模块后续接入。
6. **启动前乱码**：仍未确定原因；面板会忽略非 JSON 行，但不能说已经根治。
7. **USB 拔出同时给 ESP32 断电**：当前 USB 调试模式下，USB 拔出会重启 ESP32；“链路丢失但 ESP32 仍带电”的看门狗场景本报告用“端口保持打开、停止发命令”间接验证。等以后用电池给 ESP32 供电后，应再做一次真实链路丢失测试。

## 6. 结论

本阶段的**电脑 ↔ ESP32 USB 通信、DRY_RUN 固件和调试面板**已完成并经过：

- 主机协议测试；
- 假串口客户端 + 面板状态测试（19 项）；
- 真实 ESP32 自动验收（20 项通过，0 失败）；
- 真实面板 ARM/方向/零速/STOP 联调；
- 新面板布局（左侧控制、右侧目标与日志）自检和联调；
- STOP 后迟到 ARM/DRIVE 应答、迟到 ARMED/DISARMED 状态、断线显示残留的离线测试。

摄像头已打通 OpenCV 取图路径（本机 Camo 走 MSMF 编号 0，实际 1280x720，约 28 FPS；设备名枚举确认为 Camo），并已集成到面板“摄像头”页（只读预览，不发送运动）。Camo 免费版上限 720p/30；1080p 需要 Pro 或换 USB 摄像头。固定俯视安装、场地标定和 ArUco 仍未完成。两项人工 USB 拔插/EN 复位验收仍待用户操作。电机、驱动板、供电和视觉闭环不在本阶段范围内。
