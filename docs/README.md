# 文档索引

项目入口见 [根目录 README](../README.md)，当前状态与待验收项统一见 [交接说明](../交接说明.md)。

## 操作与开发

- [手机接入与自主清扫手册](手机接入与自主清扫手册.md) 提供现场操作流程与故障处理。
- [常用命令](常用命令.md) 提供终端命令、批处理与离线检查。
- [Windows 客户端](../windows/README.md) 说明取流、感知、任务和配置。
- [硬件与底盘](../hardware/README.md)、[本车固件](../hardware/firmware/README.md) 说明接线、版本和执行链路。
- [工具与本地环境](../tools/README.md) 说明面板、PlatformIO 编译与烧录。
- [红块训练](../training/README.md)、[数据索引](../windows/test_data/README.md)、[模型推理](../windows/models/README.md) 说明数据与检测器。
- [自主策略设计](自主策略设计.md) 说明 Windows 策略和保留的 ROS 仿真设计。

## 汇报

汇报材料按当时版本保留，当前实机状态以根目录交接说明为准。

- 第二次汇报包含 [PPT](PRE_2/Group01-2.pptx) 和 [汇报材料](PRE_2/汇报材料.md)。
- 第三次汇报包含 [PPT](PRE_3/Group01-3.pptx)、[完整讲稿](PRE_3/第三次汇报_讲稿.md) 和 [中文简稿](PRE_3/第三次汇报_中文简稿.md)。
- 第四次汇报包含 [PPT](PRE_4/Group01-4.pptx)、[中文讲稿](PRE_4/第四次汇报_中文讲稿.md) 和 [英文讲稿](PRE_4/第四次汇报_英文讲稿.md)。

## 素材与本地证据

- [实车照片](media/实车图.jpg)、[车身框架](media/车身框架.png)、[旧车架直走与后退录像](media/直走与后退.mp4)。
- [第四次汇报素材索引](media/report4/README.md) 对应照片、视频和验证记录。
- `log/` 保存本机验证证据，`hardware/workbench/` 保存封存工作台，二者由 Git 忽略。训练原图、标注和 ONNX 模型随仓库保存。
