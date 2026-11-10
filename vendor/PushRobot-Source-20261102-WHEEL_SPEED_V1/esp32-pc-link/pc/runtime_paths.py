"""统一管理源码版、打包版的配置与导出路径。"""
from pathlib import Path
import os
import sys


def data_root():
    """打包后把个人数据放在 EXE 旁的 data；源码版保留原目录。"""
    # 测试或团队自定义部署可显式指定数据目录，避免写入打包资源。
    override = os.environ.get("ROBOT_WORKBENCH_DATA_DIR", "").strip()
    if override:
        return Path(override).expanduser().resolve()
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent / "data"
    return Path(__file__).resolve().parents[2] / "debug-workbench"


def settings_file():
    return data_root() / "settings.json"


def records_directory():
    return data_root() / "records"


def camera_config_file():
    # 两种面板共用同一个相机配置；源码版仍使用原 pc/camera_config.json。
    if getattr(sys, "frozen", False) or os.environ.get("ROBOT_WORKBENCH_DATA_DIR", "").strip():
        return data_root() / "camera_config.json"
    return Path(__file__).resolve().with_name("camera_config.json")
