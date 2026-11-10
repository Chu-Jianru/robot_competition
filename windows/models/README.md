# 红块检测模型（推理）

推理模型由训练脚本导出，文件名为 `red_block.onnx`，存放于：

```text
windows/models/red_block.onnx
```

## 使用

```powershell
# CPU
python windows\detect_onnx.py --model windows\models\red_block.onnx --device cpu

# GPU（先双击 windows\安装GPU推理.cmd 安装 onnxruntime-gpu）
python windows\detect_onnx.py --model windows\models\red_block.onnx --device gpu

# 实跑预览（auto：有 CUDA 时使用 GPU）
python windows\run_task.py --preview
```

## GPU 运行时

本机无系统级 CUDA；PyTorch 自带 CUDA 12 + cuDNN 9 运行时。`detect_onnx.ensure_runtime_paths()` 将 conda 环境中的 `torch\lib` 加入 DLL 搜索路径，再加载 `onnxruntime-gpu`，无需单独安装 CUDA Toolkit。`providers` 中没有 `CUDAExecutionProvider` 时，说明当前为 CPU 版或 cuDNN 版本不匹配。

## 依赖与切换

- CPU 版依赖位于 `.deps`，GPU 版位于 `.deps-gpu`；脚本会将两者加入 `sys.path`，无需设置 `PYTHONPATH`。
- ONNX 的 precision/recall/F1 不低于 HSV 且单帧 < 100 ms 时，才将 `run_task` 的默认 `--detector` 改为 `onnx`；否则保留 HSV。
- `best.pt` 为训练产物，不放入此目录。
