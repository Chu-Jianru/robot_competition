# 红块检测训练

使用 YOLOv8n 训练单类 `red_block`，导出 ONNX 到 `windows/models/red_block.onnx`。原图、标注与来源清单见 [数据索引](../windows/test_data/README.md)。

## 当前记录

数据集记录为 225 张、4789 个框，全量导出为 182 训练 / 43 验证（`--val-every 5`）。2026-11-11 的 117 轮训练因旧缓存实际只用了 61 张旧数据，最好轮为 87；其验证 precision 0.923、recall 0.913、mAP50 0.906、mAP50-95 0.353。该次 ONNX 已复制到项目模型目录。

后续入口已增加标注同步和缓存清理。全量重训、独立场景评估及 HSV/ONNX 正式对比仍待完成。历史裸 HSV 基线为 74 张人工标注、IoU 0.3，precision 0.947、recall 0.554、F1 0.699、单帧 20.9 ms，调用条件与场地标定链路有差异。

## 采集、标注与导出

在仓库根目录使用现有环境，逐项执行所需操作。

```powershell
$robotPython = 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe'
& $robotPython .\windows\collect_red_frames.py --scrcpy-dir 'E:\scrcpy\scrcpy-win64-v5.0.1' --manual --out .\windows\test_data\red_train_20261110
& $robotPython .\windows\label_boxes.py .\windows\test_data\red_train_20261110\frames
& $robotPython .\windows\export_yolo.py .\windows\test_data\red_train_20261110 --val-every 5
```

采集会打开相机，S 存帧、Q 退出，默认按蓝黄地面自动确定朝向。每批改变块堆、光照和遮挡，补充空场、只有车和红轮的负样本。取流客户端同时只开一个。

标注时左键拖框、右键删框，Z 撤销，D 清空，N/空格/V/Enter 确认并下一张，U 取消确认，P 上一张，S 保存，Q 退出，+/- 缩放。只框可见块，空场直接确认成零框；未确认帧不导出。`frames/labels.json` 为权威标注，`yolo/` 为可重新生成的导出产物。

双击入口为 `windows/红块采帧.cmd`、`windows/红块框选标注.cmd`，后者可拖入其他 `frames` 目录。手机直拍导入用 `import_photos.py --src <照片目录> --out <数据集目录>`；批次合并用 `merge_dataset.py`，中心点转框用 `import_points.py`，具体参数用各脚本 `--help` 查看。推算框仍需人工确认。

## 本机 GPU 训练

本机 RTX 4060 Laptop 8 GB，已配置 conda `pytorch_stable`（torch 2.9.0+cu126）。训练仅在本机执行。

```powershell
& '.\windows\训练红块模型.cmd'
```

`train_launcher.py` 依次探测 `pytorch_stable`、`pytorch_gpu` 和当前解释器，选择 CUDA 可用的环境，并在 `.deps` 检查会遮蔽 CUDA 版的 torch 包。缺失补充依赖时按入口逻辑安装。

`train_local.py` 在标注比导出新时重新导出，删除标签缓存并记录数据集指纹。默认起点为 `training/yolov8n.pt`。训练权重保存在 `training/runs/<本次运行>/weights/best.pt`，结束后自动导出并复制 ONNX。权重、zip、运行目录与依赖环境均由 Git 忽略。

备用 Colab 流程为 `windows/打包训练集.cmd` 生成 `training/red_train_yolo.zip`，按 [colab_train.ipynb](colab_train.ipynb) 上传并训练，下载 ONNX 后放回模型目录。

## 检测器对比

```powershell
& '.\windows\比较红块检测.cmd'
```

该入口比较同图人工框上的 precision、recall、F1 与单帧耗时。直接调用可用 `evaluate_detector.py --frames <frames目录> --detector hsv|onnx`，ONNX 另传 `--model windows/models/red_block.onnx`。

任务当前默认 `auto`，按模型存在与否选择。比赛检测器的选择需依据对比结果，目标为 F1 不低于 HSV 且单帧小于 100 ms。聚堆、墙边、遮挡、反光、空场和只有车的场景仍需单独核对。当前数据来自开发期，验证划分及训练分数只用于对应数据条件下的评估。
