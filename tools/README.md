# 工具与本地环境

这里是**不进入交付包、也不属于 vendor** 的本地工具目录：解释器、编译产物、启动入口、排查记录。全部可删可重建。

日常操作可双击 [日常工具.cmd](日常工具.cmd) 打开中文数字菜单，菜单逻辑在 `windows/daily_tools.py`。该 CMD 使用 ASCII 与 CRLF，不需要在 PowerShell 中设置变量；具体按键见 [常用命令](../docs/常用命令.md)。菜单本身不自动连接设备，选择对应操作后才启动工具。

## 目录

| 路径 | 用途 | 是否入库 |
|---|---|---|
| `panel-venv/` | 新版面板的 Python 环境 | 否（`.gitignore`） |
| `panel-requirements.txt` | 面板依赖（锁版本） | 是 |
| `panel-data/` | 面板的 `settings.json` 与 `records/` | 否 |
| `pio-venv/` | PlatformIO Core 环境 | 否 |
| `pio-core/` | PlatformIO 平台、工具链、包 | 否 |
| `pio-build/` | 固件构建产物（`esp32dev/firmware.bin`） | 否 |
| `pio-libdeps/` | PlatformIO 库依赖 | 否 |
| `check-firmware-sync.ps1` | 核对工作副本与原始 zip 是否一致 | 是 |
| `核对固件版本.cmd` | 上面脚本的双击入口 | 是 |
| `启动新面板.cmd` | 双击启动 2026-11-02 面板 | 是 |

**不要保留第二份 esptool 环境**：PlatformIO 的 `pio-core/packages/tool-esptoolpy/` 已经带 esptool 4.11，烧录和探测都用它，单独再建一个 venv 只是重复。

本地脚本一律**纯 ASCII + CRLF**：`.cmd` 和 `.ps1` 若存成带中文的 UTF-8，cmd 会按 GBK 解码、PowerShell 5.1 也会按 ANSI 读，轻则乱码重则语法错误。

## 启动新版面板

```powershell
tools\启动新面板.cmd
```

脚本自己会：检查依赖 → 设置 `PYTHONPATH`（`debug-workbench`、`esp32-pc-link/pc`、`motor-bringup` 三处）→ 把数据目录重定向到 `tools\panel-data` → 关掉字节码写入，避免污染 `vendor/`。

缺依赖时的重装命令：

```powershell
python -m venv tools\panel-venv
tools\panel-venv\Scripts\python.exe -m pip install -r tools\panel-requirements.txt --index-url https://pypi.org/simple
```

**必须用官方源**：清华镜像没有 `Pillow>=12`，用默认源会报 `Could not find a version that satisfies the requirement Pillow<14,>=12`。

## 编译与烧录固件

```powershell
$env:PLATFORMIO_CORE_DIR = "$PWD\tools\pio-core"
$env:PLATFORMIO_BUILD_DIR = "$PWD\tools\pio-build"      # 不要再带 \esp32dev，否则会嵌套一层
$env:PLATFORMIO_LIBDEPS_DIR = "$PWD\tools\pio-libdeps"
tools\pio-venv\Scripts\pio.exe run -d hardware\firmware\WHEEL_SPEED_V1\motor-bringup
```

产物固定在 `tools\pio-build\esp32dev\firmware.bin`，2026-11-02 的镜像 SHA256 为 `EB4978344A49CDBD56BBFC43CF4857188047CD58E0FF3410E82565CE77BF802C`。

烧录（USB，端口通常是 COM3）：

```powershell
tools\pio-venv\Scripts\python.exe tools\pio-core\packages\tool-esptoolpy\esptool.py `
  --chip esp32 -p COM3 -b 115200 --before no_reset --after hard_reset write_flash -z `
  --flash_mode dio --flash_freq 40m --flash_size detect `
  0x1000 tools\pio-build\esp32dev\bootloader.bin `
  0x8000 tools\pio-build\esp32dev\partitions.bin `
  0xe000 tools\pio-core\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin `
  0x10000 tools\pio-build\esp32dev\firmware.bin
```

`--before no_reset` 要求芯片已在下载模式（按住 BOOT 点一下 EN 再松开）。自动复位在这种板子上不稳定，会报 `Wrong boot mode detected` 或 `Invalid head of packet`。

## 已知坑（都踩过）

| 现象 | 原因 | 处理 |
|---|---|---|
| `flash read err, 1000` 循环重启 | 板上没有可启动固件，ROM 反复复位 | 进下载模式后 `write_flash` |
| `flash-id` 读到 `ff/ffff` | 芯片还在复位循环，Flash 通信被打断 | 手动 BOOT+EN 后加 `--before no_reset` |
| `Wrong boot mode detected (0x13)` | 没进下载模式 | 手动 BOOT+EN |
| `Invalid head of packet (0x42)` | 收到的是启动日志字符 `B` | 同上 |
| 设置里显示“已配对”但面板超时 | 先核对是否选了蓝牙传入端口；不能仅凭地址 `…962C` / `…962E` 不同认定是另一块板 | ESP32 默认四地址配置中，蓝牙地址是基础 MAC +2；本次 `…962E` 的传出 COM6 已握手成功，无须因此删配对 |
| 面板下拉框里一堆 COM | 2026-11-03 本次配对为 COM6 传出、COM7 传入；重新配对可能重新编号 | 选“更多蓝牙设置 → COM 端口”中对应设备的传出端口；USB 另看 CP210x 描述 |
| `.cmd` 一运行就报奇怪命令 | 用 UTF-8 存了中文注释，cmd 按 GBK 解码 | 启动脚本一律纯 ASCII + CRLF |
| 往 `hardware/` 写文件被拒 | 本机 DACL 没有把沙箱权限继承到子目录 | 用编辑工具写，或按提示提升访问权限 |
