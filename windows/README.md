# Windows 实机客户端

手机后置摄像头经 USB/scrcpy 取流，PyAV 解码，OpenCV/ONNX 识别场地、小车和红块，策略通过蓝牙 SPP 控制 ESP32。Python 3.12.2 与 scrcpy 5.0.1 的现有环境位于 `E:\scrcpy\scrcpy-win64-v5.0.1`。

日常使用双击 [日常工具](../tools/日常工具.cmd)。现场步骤、按键和故障处理统一见 [手机接入与自主清扫手册](../docs/手机接入与自主清扫手册.md)，终端调用见 [常用命令](../docs/常用命令.md)，进度见 [交接说明](../交接说明.md)。

## 模块

- [daily_tools.py](daily_tools.py) 组织菜单、USB 授权与取流占用检查。
- [usb_camera.py](usb_camera.py) 提供 scrcpy 采集，保留最新帧；[capture_hi.py](capture_hi.py) 提供预览、拍照和标定入口。
- [field_view.py](field_view.py) 保存四角透视标定；[pose_tag.py](pose_tag.py)、[tag_detector.py](tag_detector.py) 解码车顶标记并计算位置与朝向。
- [field_vision.py](field_vision.py) 负责地面分界、场内范围、红块归属和障碍提取；[calibrated_blocks.py](calibrated_blocks.py)、[red_blocks.py](red_blocks.py) 提供 HSV 检测。
- [onnx_blocks.py](onnx_blocks.py)、[detect_onnx.py](detect_onnx.py) 提供模型选择和推理；[block_tracks.py](block_tracks.py) 做重复合并、确认、平滑与短时保持。
- [run_task.py](run_task.py) 连接视觉、标定、策略与底盘；[sweep_policy.py](sweep_policy.py) 实现横推归拢、沿墙送出和逐群清理，复用 `src/sweep_robot/sweep_robot/competition_policy.py`。
- [chassis_link.py](chassis_link.py) 负责握手、速度流、心跳与停车；[keyboard_drive.py](keyboard_drive.py)、[remote_view.py](remote_view.py) 提供遥控及检测窗口。
- [record_diag.py](record_diag.py)、[live_bench.py](live_bench.py) 记录连续帧与台架统计；[detect_field_batch.py](detect_field_batch.py)、[evaluate_field_batch.py](evaluate_field_batch.py) 处理照片批次。

## 配置与数据

- `field_calib.json` 为本机场地四角标定。菜单取流使用 `--rotate 90 --max-size 1280`，本机为 960×1280。机位、旋转、变焦或分辨率变化后重新标定。
- `task_drive.json` 为本车运动换算、端口与轮位配置，由菜单 11 生成。旧版 PWM 参数需要重新标定到编码器计数单位。
- `detection_flags.json` 保存菜单 14 的障碍开关，缺失时默认开启。
- `red_block_profile.json` 保存 HSV 配置。车顶 tag 默认读取 `tag_detector.TAG_ID`，当前为 `DICT_4X4_50`、ID 7、黑框 50 mm。
- [tags/aruco_4x4_50_50mm_A4.pdf](tags/aruco_4x4_50_50mm_A4.pdf) 为打印图纸，[make_tags.py](tags/make_tags.py) 为生成器。打印保持实际尺寸，标记上边朝车头，贴平并减少反光。
- [test_data/](test_data/README.md) 保存原图、标注与清单，[models/](models/README.md) 保存推理模型。`captures_hi/` 保存现场照片与诊断输出。

场地标定、运动参数、检测开关和新采集截图均为本机数据，由 Git 忽略。已有入库回归截图继续随仓库保存。

## 检测器

`onnx_blocks.build_detector()` 是任务、台架与连续帧诊断的共用选择入口。`--detector auto` 在模型文件存在时使用 ONNX，缺失时使用 HSV；`--detector hsv` 强制 HSV，`--detector onnx` 显式要求模型。`--onnx-device auto|gpu|cpu` 选择任务推理后端，启动日志显示实际后端。

场地模式经透视校正、围栏内沿裁剪、归属判断和重叠去重后交给策略；台架模式用 tag 尺度做毫米筛选。二者共用检测器，处理条件各有范围。旧照片的曝光补检与批处理工具仍用于离线回归。

模型训练、数据导入、标注与对比统一见 [训练说明](../training/README.md)。模型与 HSV 的正式对比和实机场景验收仍待完成。

## 依赖与离线检查

已有环境可直接使用。新环境安装 [requirements.txt](requirements.txt)，ONNX CPU 推理另装 `onnxruntime`；本机 GPU 配置见 [模型说明](models/README.md)。

在仓库根目录执行 Windows 离线测试，使用照片、合成输入与假串口。

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' -B -m unittest discover -s windows -p 'test_*.py'
```

## 保留的联调工具

[nav_servo.py](nav_servo.py) 为定时 `MECANUM` 脉冲联调入口，含独立纯 Python `--sim` 模式。其 `mm_per_percent_second`、`deg_per_percent_second` 默认值仍需实测；连续轮速闭环任务由 `run_task.py` 执行，运动配置由菜单 11 保存。

ROS 适配位于 `src/`，Gazebo 模型位于 `competition_robot/`，h15 仿真保持暂停。现场标定与完整比赛能力的验收记录统一维护在根目录交接说明中。
