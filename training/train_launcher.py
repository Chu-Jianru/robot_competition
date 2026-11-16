"""训练入口：自动找一个能用 CUDA 的 Python，再调用 train_local.py。

为什么要多这一层：这台机器上 PyTorch 装在 conda 环境里，项目 venv 里没有。
批处理里写复杂的判断很容易出错（cmd 对括号块和标签很挑），所以把逻辑放这里，
批处理只负责“切目录 + 调用我”。

用法：
    python training\\train_launcher.py            # 自动选环境并训练
    python training\\train_launcher.py --epochs 40 --imgsz 960
    python training\\train_launcher.py --python C:\\...\\python.exe   # 指定解释器
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
TRAIN = HERE / "train_local.py"
DEPS = ROOT / ".deps"
CANDIDATES = [
    Path(r"C:\Users\chuji\anaconda3\envs\pytorch_stable\python.exe"),
    Path(r"C:\Users\chuji\anaconda3\envs\pytorch_gpu\python.exe"),
]


def probe(python: Path) -> str:
    """Return 'cuda', 'cpu' or 'none' for one interpreter."""
    if not python.exists():
        return "none"
    try:
        result = subprocess.run(
            [str(python), "-c",
             "import torch,sys; print('cuda' if torch.cuda.is_available() else 'cpu')"],
            capture_output=True, text=True, timeout=120)
    except Exception:                                          # noqa: BLE001
        return "none"
    if result.returncode != 0:
        return "none"
    return "cuda" if "cuda" in result.stdout else "cpu"


def has_packages(python: Path) -> bool:
    result = subprocess.run([str(python), "-c", "import ultralytics, onnxruntime"],
                            capture_output=True, text=True,
                            env=with_deps(os.environ))
    return result.returncode == 0


# 这些包必须用 conda 环境里的 CUDA 版本，绝不能由 .deps 覆盖。
# pip install --target .deps ultralytics 会把 torch 当依赖一起装进来，
# 而默认源给的是 CPU 轮子，PYTHONPATH 一前置就把 conda 的 CUDA 版整个盖掉。
NO_SHADOW = ("torch", "torchvision", "torchaudio", "functorch", "torchgen",
             "nvidia", "triton")


def shadowing_packages():
    """Return the NO_SHADOW entries that .deps currently contains."""
    if not DEPS.is_dir():
        return []
    return [name for name in NO_SHADOW if (DEPS / name).exists()]


def with_deps(env):
    """Put the repo-local dependency folder first on PYTHONPATH.

    Refuses to do so when .deps holds a shadowing torch: that would silently
    downgrade a working CUDA build to CPU, which is exactly the failure this
    guard exists to prevent.
    """
    shadowing = shadowing_packages()
    if shadowing:
        raise RuntimeError(
            f"{DEPS} 里有 {', '.join(shadowing)}，会覆盖 conda 的 CUDA 版。\n"
            f"请先删除这些目录后重试，例如：\n"
            f"  Remove-Item -Recurse -Force {DEPS}\\torch, {DEPS}\\torchvision")
    merged = dict(env)
    parts = [str(DEPS)]
    if merged.get("PYTHONPATH"):
        parts.append(merged["PYTHONPATH"])
    merged["PYTHONPATH"] = os.pathsep.join(parts)
    return merged


def install_deps(python: Path) -> int:
    """Install training packages into .deps so the conda env stays untouched.

    torch/torchvision come from the conda environment (CUDA build), so they are
    named as already-satisfied requirements: pip still resolves ultralytics, but
    does not download a second, CPU-only torch into .deps.
    """
    DEPS.mkdir(exist_ok=True)
    command = [str(python), "-m", "pip", "install", "--target", str(DEPS),
               "--upgrade-strategy", "only-if-needed",
               "ultralytics", "onnx", "onnxruntime"]
    print("安装到项目内 .deps（不改 conda 环境）：")
    print("  " + " ".join(command), flush=True)
    return subprocess.call(command)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--python", type=Path, help="手动指定解释器，跳过自动选择")
    parser.add_argument("--no-install", action="store_true", help="缺包时只提示，不自动安装")
    known, extra = parser.parse_known_args()

    chosen = None
    if known.python:
        chosen = known.python
        print(f"使用指定解释器：{chosen}")
    else:
        for candidate in CANDIDATES:
            state = probe(candidate)
            print(f"检查 {candidate}: {state}")
            if state == "cuda" and chosen is None:
                chosen = candidate
        if chosen is None:
            state = probe(Path(sys.executable))
            print(f"检查当前解释器 {sys.executable}: {state}")
            if state == "cuda":
                chosen = Path(sys.executable)
            elif state == "cpu":
                print("没有可用的 CUDA，只能用 CPU 训练（很慢）。继续请加 --python 指定。")
                return 2
    if chosen is None:
        print("找不到带 CUDA 的 PyTorch。请确认 conda 环境名，或用 --python 指定解释器。",
              file=sys.stderr)
        return 2
    print(f"训练解释器：{chosen}")

    # 装完依赖再复查一次 CUDA：.deps 里若混进 CPU 版 torch，会在这里被拦住，
    # 而不是等到 train_local.py 打印一句“CUDA 不可用”就悄悄降级。
    shadowing = shadowing_packages()
    if shadowing:
        print(f"\n{DEPS} 里有 {', '.join(shadowing)}，会覆盖 {chosen} 的 CUDA 版 PyTorch。",
              file=sys.stderr)
        print("这些包不该装进 .deps。删除后重试：", file=sys.stderr)
        for name in shadowing:
            print(f"  Remove-Item -Recurse -Force '{DEPS / name}'", file=sys.stderr)
        return 2

    if not has_packages(chosen):
        print(f"{chosen} 里没有 ultralytics/onnxruntime。")
        if known.no_install:
            print("请手动安装到 .deps（见 training/README.md）。", file=sys.stderr)
            return 2
        code = install_deps(chosen)
        if code != 0:
            print("安装失败；检查网络后重试。", file=sys.stderr)
            return code
        # pip 可能又把 torch 拉进 .deps，装完必须再查一次。
        shadowing = shadowing_packages()
        if shadowing:
            print(f"\n安装时把 {', '.join(shadowing)} 带进了 .deps，这会覆盖 CUDA 版 PyTorch。",
                  file=sys.stderr)
            print("已中止。删除后重试：", file=sys.stderr)
            for name in shadowing:
                print(f"  Remove-Item -Recurse -Force '{DEPS / name}'", file=sys.stderr)
            return 2
        if not has_packages(chosen):
            print("装完仍导入失败；把上面的错误发我。", file=sys.stderr)
            return 2

    command = [str(chosen), str(TRAIN)] + extra
    print("开始训练：" + " ".join(command), flush=True)
    return subprocess.call(command, env=with_deps(os.environ))


if __name__ == "__main__":
    raise SystemExit(main())

