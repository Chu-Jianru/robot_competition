"""菜单 10/11/12/13 共用的检测开关，菜单 14 可切换。"""
from __future__ import annotations

import json
from pathlib import Path

PATH = Path(__file__).resolve().with_name("detection_flags.json")
DEFAULTS = {"obstacles": True}


def load(path: Path = PATH) -> dict:
    """读开关文件；缺失或损坏时返回默认值，未知键原样保留。"""
    flags = dict(DEFAULTS)
    try:
        saved = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return flags
    if isinstance(saved, dict):
        flags.update(saved)
    return flags


def obstacles_enabled(path: Path = PATH) -> bool:
    return bool(load(path)["obstacles"])


def save(flags: dict, path: Path = PATH) -> None:
    path = Path(path)
    pending = path.with_suffix(path.suffix + ".tmp")
    pending.write_text(json.dumps(flags, indent=2, ensure_ascii=False) + "\n",
                       encoding="utf-8")
    pending.replace(path)


def set_obstacles(enabled: bool, path: Path = PATH) -> bool:
    """只改障碍开关，保留文件里的其它键；返回写入后的状态。"""
    flags = load(path)
    flags["obstacles"] = bool(enabled)
    save(flags, path)
    return flags["obstacles"]
