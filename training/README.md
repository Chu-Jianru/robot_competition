# 红块检测训练

数据在 `windows/test_data/red_train_20261110`（74 张、1326 个框、单类 `red_block`，
导出后 61 训练 / 13 验证）。本目录放训练侧脚本；Windows 上负责采帧、标注、导出、
训练和最终推理。

## 当前状态（2026-11-10）

本机已完成一次 111 轮训练（`training/runs/red_block/args.yaml` 记录 `device: cpu`、
`imgsz: 1280`），验证集 precision 0.930 / recall 0.924 / mAP50 0.930 / mAP50-95 0.390，
`best.onnx` 已导出到 `windows/models/red_block.onnx`（约 13 MB）。这是开发集的同分布
验证，不是独立测试集，也**还没有**完成与 HSV 在同一批人工框上的对比；不要据此宣称
ONNX 已超过 HSV 或已通过实机验收。

`training/yolov8n.pt`（起点权重）和 `training/red_train_yolo.zip`（Colab 上传包）是
本地文件，已在 `.gitignore` 排除；`training/runs/` 同样是本地产物。

## 路线 A：本机 GPU（推荐）

这台机器有 **RTX 4060 Laptop 8 GB**，PyTorch 已经在 conda 环境里（实测
`pytorch_stable`: torch 2.9.0+cu126，CUDA 可用），所以不用重装 PyTorch：

```powershell
# 双击即可
windows\训练红块模型.cmd
```

`train_launcher.py` 会：

1. 依次探测 `pytorch_stable`、`pytorch_gpu` 和当前解释器，选第一个 CUDA 可用的；
2. 缺 `ultralytics/onnxruntime` 时，用 `pip install --target .deps` 装到**项目内**，
   **不改你的 conda 环境**；
3. 带上 `.deps` 的 `PYTHONPATH` 启动训练。

想手动指定解释器或跳过快看趋势：

```powershell
& 'C:\Users\chuji\anaconda3\envs\pytorch_stable\python.exe' training\train_launcher.py
& '...\python.exe' training\train_launcher.py --epochs 40 --imgsz 960
& '...\python.exe' training\train_launcher.py --python C:\path\to\python.exe
```

产物：

- `training/runs/<本次运行>/weights/best.pt`：继续训练用（本地，不入库）
- `windows/models/red_block.onnx`：给车上推理用（脚本会自动导出并复制过去）

> 说明：本机有 NVIDIA RTX 4060 Laptop 8 GB，可以本地训练；训练仍只在本机自己跑，
> 不在 DSH 沙箱里跑（安装和训练都需要写 `E:\scrcpy` 下的 venv）。
> `training/yolov8n.pt` 是起点权重，已从仓库根目录移到这里；`train_local.py`
> 的默认 `--weights` 指向它。

## 路线 B：Colab（没有本机 GPU 时）

1. `windows\打包训练集.cmd` 生成 `training\red_train_yolo.zip`；
2. 打开 [colab_train.ipynb](colab_train.ipynb)，上传本子；
3. **代码执行程序 → 更改运行时类型 → T4 GPU**；
4. 全部运行，中间上传 zip；
5. 下载 `best.onnx`，改名 `red_block.onnx` 放到 `windows\models\`。

## 训练后必须做的对比

```powershell
windows\比较红块检测.cmd
```

它并排跑 HSV 与 ONNX，输出 precision / recall / F1 / 单帧毫秒。**只有 ONNX 的 F1
不低于 HSV、单帧 < 100 ms，才把 `run_task` 的默认 `--detector` 改成 `onnx`**，
并保留 `--detector hsv` 回退。

截至 2026-11-10，这个对比还没有保存正式结果；`run_task` 代码里的 `auto` 只是
“有模型就用”，不等于已经验收超过 HSV。

当前 HSV 基线（74 张人工标注，IoU 0.3）：

| 指标 | 值 |
|---|---|
| precision | 0.947 |
| recall | 0.554 |
| F1 | 0.699 |
| 单帧 | 20.9 ms |

## 已知的数据短板

- **没有负样本**：74 张里每张都有块，没有“空场 / 只有车 / 只有红轮”的帧。
  上线后车把块推空时可能继续误检。建议补 10–15 帧空场和“只有车”的帧，标 0 框并确认。
- **尺度跨度大**：手机照片 17–42 px、取流帧 9–17 px。靠 `scale=0.6` 和多尺度补，
  但 9 px 已接近检测极限。
- 数据来自开发期，不是独立测试集；`--val-every 5` 只防止相邻帧泄漏。
