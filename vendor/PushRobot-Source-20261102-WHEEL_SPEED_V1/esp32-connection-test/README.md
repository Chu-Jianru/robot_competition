# ESP32 USB 串口连接测试

适用假设：ESP32-WROOM-32 常见开发板，使用 PlatformIO 的通用 esp32dev 配置。
这不是对 H12051705 各版本板载 USB 串口芯片及引脚布局的确认。

## 使用

1. VS Code 扩展中安装 PlatformIO IDE，等待初始化完成，必要时重启 VS Code。
2. 在 VS Code 中打开本目录（包含 platformio.ini 的目录）。
3. 暂时断开电机、驱动板和其他外设，仅使用支持数据传输的 USB 线连接开发板与电脑。
4. 在设备管理器中比较拔插前后的“端口（COM 和 LPT）”，找出开发板对应的 COM 口。
5. 在 PlatformIO 中执行 Build，然后执行 Upload。
6. 打开 Serial Monitor，波特率设为 115200；必要时按开发板 EN/RESET。
7. 每秒应出现 ESP32 OK 和递增计数。输入大写 PING 后回车，开发板会回复 PONG。

## 多个串口时

如果自动选择串口失败，在 platformio.ini 的现有配置节下添加两行。
将 COM7 替换为实际开发板端口：

```ini
upload_port = COM7
monitor_port = COM7
```

## 常见问题

- 没有新增 COM 口：先换确认可传数据的 USB 线和 USB 接口，再按板上实际 USB 转串口芯片安装厂商驱动。
- 上传停在 Connecting：按住 BOOT 开始上传，出现写入进度后松开；结束后按 EN/RESET。
- 串口被占用：关闭其他串口工具或已有的监视器，再上传。
- 串口乱码：确认监视器波特率为 115200。
- 电源灯亮只代表有供电；连续计数和 PING/PONG 通信才是本测试的通过条件。

此测试不操作 GPIO，也不测试 Wi-Fi 或电机。
