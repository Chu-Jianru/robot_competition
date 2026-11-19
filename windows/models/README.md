# 红块推理模型

[red_block.onnx](red_block.onnx) 由 YOLO 训练导出，随仓库保存。模型来源、训练数据限制与检测器对比见 [训练说明](../../training/README.md)。任务入口 `--detector auto` 在模型存在时使用 ONNX，缺失时回退 HSV。

## 本机推理

在项目根目录使用现有 Python 环境做照片离线推理。

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\detect_onnx.py --model .\windows\models\red_block.onnx --device cpu --limit 20
```

`--device auto|gpu|cpu` 选择后端，`--save log/<本次目录>` 可保存画框图。任务入口对应参数为 `--onnx-device`。

CPU 使用 `onnxruntime`。本机 GPU 依赖位于 `.deps-gpu/`，CPU/训练补充依赖位于 `.deps/`；`detect_onnx.ensure_runtime_paths()` 按顺序加载，并将已有 PyTorch 的 CUDA/cuDNN DLL 目录加入搜索路径。

本机 GPU 配置入口为 [安装GPU推理.cmd](../安装GPU推理.cmd)。移机后按实际 CUDA/cuDNN 环境准备依赖，启动日志中的实际 provider 用于确认推理后端。
