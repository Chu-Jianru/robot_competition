# 红块检测训练

数据集位于 `windows/test_data/red_train_20261110`：225 张、4789 个框、单类 `red_block`，导出后为 182 训练 / 43 验证（`--val-every 5`）。本目录包含训练脚本；采帧、标注与导出在 `windows/` 下。

## 当前状态

2026-11-11 完成一次 117 轮训练（`training/runs/red_block_20261111_1818`，GPU，早停于第 87 轮后第 30 轮）。验证集 precision 0.923、recall 0.913、mAP50 0.906、mAP50-95 0.353；`best.onnx` 已自动导出到 `windows/models/red_block.onnx`（约 13 MB）。

> 该次训练读的是 17:52 生成的 61 张旧缓存，**未用上 225 张新数据**。之后的训练入口已自动清理缓存，重跑即可用全量数据。

早停由 `patience=30` 触发：ultralytics 的 fitness 为 `0.1×mAP50 + 0.9×mAP50-95`，最好轮次之后连续 30 轮无改进即停止，属正常行为，不代表训练失败。

该结果来自开发集同分布验证，不是独立测试集；与 HSV 在同一批人工框上的对比尚未完成。ONNX 超过 HSV 或通过实机验收均无数据支持。

`training/yolov8n.pt`（起点权重）和 `training/red_train_yolo.zip`（Colab 上传包）为本地文件，已由 `.gitignore` 排除；`training/runs/` 同样为本地产物。

## 本机 GPU

本机 GPU 为 NVIDIA RTX 4060 Laptop 8 GB。PyTorch 位于 conda 环境 `pytorch_stable`（torch 2.9.0+cu126，CUDA 可用），无需重装。

```powershell
windows\训练红块模型.cmd
```

`train_launcher.py` 的处理顺序：

1. 依次探测 `pytorch_stable`、`pytorch_gpu` 和当前解释器，选择第一个 CUDA 可用的；
2. 检查 `.deps` 里有没有 `torch`/`torchvision`/`nvidia` 等包——有就中止并打印删除命令。`pip install --target .deps` 会把 CPU 版 torch 装进来，`PYTHONPATH` 前置后会盖掉 conda 的 CUDA 版，表现为"明明装了 GPU 却报 CUDA 不可用"；
3. 缺少 `ultralytics/onnxruntime` 时，执行 `pip install --target .deps` 安装到项目内，不修改 conda 环境，装完再查一次上一条；
4. 将 `.deps` 加入 `PYTHONPATH` 后启动训练。

`train_local.py` 在真正训练前会自动做两件事，不需要手工跑导出：

- **同步标注**：`frames/labels.json` 比 `yolo/export.json` 新时，自动调用 `export_yolo.py` 重新导出，所以"训练"永远用当前标注，不会训到旧导出上；
- **清理缓存**：删除 `yolo/labels/*.cache`，并打印数据集指纹（文件数 + 大小 + mtime 的哈希），避免复用过期缓存。

产物：

- `training/runs/<本次运行>/weights/best.pt`：继续训练使用（本地，不入库）
- `windows/models/red_block.onnx`：车上推理使用（训练结束**自动**导出并复制，无需手工改名）
- `windows/run_task.py` 的 `--detector` 默认 `auto`：模型存在即用 ONNX，启动时打印 `红块检测：ONNX …（后端 …）`；`--detector hsv` 可强制回退。

> 训练仅在本机执行，不在 DSH 沙箱内运行（安装与训练需写入 `E:\scrcpy` 下的 venv）。
> `training/yolov8n.pt` 为起点权重，已从仓库根目录移至 `training/`；`train_local.py` 的默认 `--weights` 指向该文件。

## Colab（无本机 GPU）

1. `windows\打包训练集.cmd` 生成 `training\red_train_yolo.zip`；
2. 打开 [colab_train.ipynb](colab_train.ipynb)，上传笔记本；
3. **代码执行程序 → 更改运行时类型 → T4 GPU**；
4. 全部运行，中途上传 zip；
5. 下载 `best.onnx`，改名 `red_block.onnx` 放入 `windows\models\`。

## 训练后的检测器对比

```powershell
windows\比较红块检测.cmd
```

该脚本并排运行 HSV 与 ONNX，输出 precision / recall / F1 和单帧毫秒。ONNX 的 F1 不低于 HSV 且单帧 < 100 ms 时，才将 `run_task` 的默认 `--detector` 改为 `onnx`，并保留 `--detector hsv` 回退。

截至 2026-11-10 尚无正式对比结果；`run_task` 的 `auto` 仅表示模型存在时使用 ONNX，不代表性能已超过 HSV。

当前 HSV 基线（74 张人工标注，IoU 0.3）：

| 指标 | 值 |
|---|---|
| precision | 0.947 |
| recall | 0.554 |
| F1 | 0.699 |
| 单帧 | 20.9 ms |

## 数据限制

- 负样本少：225 张几乎都含红块，缺少空场、只有车、只有红轮的帧；推空后可能继续误检。建议补 10–15 帧空场和只有车的帧，标 0 框并确认。
- 尺度跨度大：手机照片 17–42 px，取流帧 9–17 px；依赖 `scale=0.6` 和多尺度增强，9 px 已接近检测极限。
- 数据来自开发期，不是独立测试集；`--val-every 5` 仅用于减少相邻帧泄漏。
- `frames/labels.json` 是唯一权威标注；`yolo/` 是导出产物，不要手工改，改完标注重跑训练会自动重新导出。
