# 红块检测模型（推理用）

把 Colab 训练产出的 `best.onnx` 放在这里，文件名建议 `red_block.onnx`：

```
windows/models/red_block.onnx
```

使用：

```powershell
# 只用 CPU
python windows\detect_onnx.py --model windows\models\red_block.onnx --device cpu

# 用 GPU（先双击 windows\安装GPU推理.cmd 装 onnxruntime-gpu）
python windows\detect_onnx.py --model windows\models\red_block.onnx --device gpu

# 实跑预览（auto = 有 CUDA 就用 GPU）
python windows\run_task.py --preview
```

GPU 说明：这台机器没有系统级 CUDA，但 PyTorch 自带 CUDA 12 + cuDNN 9 运行时，
`detect_onnx.ensure_runtime_paths()` 会把 conda 环境里的 `torch\lib` 加进 DLL 搜索路径，
再加载 `onnxruntime-gpu`。所以不需要单独装 CUDA Toolkit。
如果 `providers` 里没有 `CUDAExecutionProvider`，说明装的还是 CPU 版或 cuDNN 版本不匹配。

- 依赖：CPU 版是 `.deps`，GPU 版装到 `.deps-gpu`，脚本会自动把两者加入 `sys.path`，不需要设 PYTHONPATH。
- 只有 ONNX 的 precision/recall/F1 **不低于 HSV**、单帧 < 100 ms，才把 `run_task` 的默认 `--detector` 改成 `onnx`；否则保留 HSV。
- `best.pt` 是训练产物，不用放到这个目录。
