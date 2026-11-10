"""构建当前源码 ZIP：按白名单选文件，不携带环境、个人设置或历史发布物。"""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import importlib.metadata as metadata
import json
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE_VERSION = "2026-11-02-WHEEL_SPEED_V1"
PACKAGE_NAME = "PushRobot-Source-20261102-WHEEL_SPEED_V1"
SELECTIONS = {
    "motor-bringup": ("platformio.ini", "*.md", "motor_link.py", "chassis_link.py",
                      "src/*.cpp", "src/*.h", "tests/*.cpp", "tests/*.h", "tests/*.py"),
    "debug-workbench": ("*.py", "README.md", "TEST-REPORT.md", "tests/*.py"),
    "esp32-pc-link": ("platformio.ini", "README.md", "TEST-REPORT.md", "src/*.cpp",
                      "lib/link_core/*", "host-tests/*.cpp", "host-tests/*.ps1", "host-tests/*.sh",
                      "pc/*.py", "pc/*.cmd", "pc/*.ps1", "pc/requirements.txt", "pc/tests/*.py"),
    "esp32-connection-test": ("platformio.ini", "README.md", "src/*.cpp"),
    "docs/superpowers": ("specs/*.md", "plans/*.md"),
    "docs/整理记录": ("源码审查与分享说明-2026-11-02.md",),
    "release_tools": ("build_source.py", "run_checks.py", "setup_source.ps1", "SOURCE_README.md",
                      "vendor-licenses/*"),
}
BLOCKED_PARTS = {".venv", ".pio", "__pycache__", ".git", "records", ".superpowers", ".claude"}


def encoded(text, batch=False):
    return text.replace("\n", "\r\n").encode("ascii") if batch else text.encode("utf-8")


def collect():
    selected = {ROOT / "run_debug.cmd"}
    for folder, patterns in SELECTIONS.items():
        for pattern in patterns:
            selected.update((ROOT / folder).glob(pattern))
    files = {}
    for path in sorted(selected):
        if not path.is_file():
            continue
        relative = path.relative_to(ROOT)
        if not path.resolve().is_relative_to(ROOT) or set(relative.parts) & BLOCKED_PARTS:
            raise ValueError(f"Unexpected source path: {relative}")
        files[relative.as_posix()] = path.read_bytes()
    files["README.md"] = (ROOT / "release_tools/SOURCE_README.md").read_bytes()
    files["setup_source.cmd"] = encoded(
        '@echo off\nsetlocal\npowershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0release_tools\\setup_source.ps1"\n'
        'if errorlevel 1 pause\n', batch=True)
    files["run_checks.cmd"] = encoded(
        '@echo off\nsetlocal\nset "PYTHONUTF8=1"\n'
        '"%~dp0esp32-pc-link\\pc\\.venv\\Scripts\\python.exe" "%~dp0release_tools\\run_checks.py" %*\n'
        'if errorlevel 1 pause\n', batch=True)
    # 统一使用默认相机配置，绝不复制发送人的设备编号或视频流地址。
    files["esp32-pc-link/pc/camera_config.json"] = encoded(
        json.dumps({"source": 0, "backend": "msmf", "width": 1280, "height": 720}, indent=2) + "\n")
    files[".gitignore"] = encoded(
        ".venv/\n.pio/\n__pycache__/\n*.pyc\n*.exe\ntmp/\noutput/\n"
        "debug-workbench/settings.json\ndebug-workbench/records/\n"
        "esp32-pc-link/pc/camera_config.json\nesp32-pc-link/pc/smoke_results.json\n")
    versions = {}
    for name in ("pyserial", "opencv-python", "Pillow", "numpy", "pygrabber", "comtypes"):
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    files["environment-snapshot.json"] = encoded(json.dumps(
        {"python": sys.version.split()[0], "packages": versions,
         "note": "Packaging environment only; verification results are recorded separately."}, indent=2) + "\n")
    required = ("motor-bringup/src/main.cpp", "motor-bringup/src/wheel_speed.h",
                "esp32-pc-link/lib/link_core/link_core.cpp", "debug-workbench/main.py",
                "esp32-pc-link/pc/requirements.txt", "run_debug.cmd")
    for name in required:
        if name not in files:
            raise ValueError(f"Required source missing: {name}")
    return files


def verify_archive(archive):
    """核对 ZIP 内每个文件与清单；不依赖本机工作区的其余文件。"""
    with zipfile.ZipFile(archive) as bundle:
        if bundle.testzip() is not None:
            raise ValueError("ZIP CRC verification failed")
        names = bundle.namelist()
        prefix = PACKAGE_NAME + "/"
        if len(names) != len(set(names)):
            raise ValueError("Duplicate ZIP entries")
        for name in names:
            parts = PurePosixPath(name).parts
            if not name.startswith(prefix) or ".." in parts or set(parts) & BLOCKED_PARTS:
                raise ValueError(f"Unexpected ZIP entry: {name}")
        manifest = json.loads(bundle.read(prefix + "source-manifest.json"))
        expected = {prefix + entry["path"] for entry in manifest["files"]} | {prefix + "source-manifest.json"}
        if expected != set(names):
            raise ValueError("ZIP file list does not match manifest")
        for entry in manifest["files"]:
            content = bundle.read(prefix + entry["path"])
            if len(content) != entry["bytes"] or hashlib.sha256(content).hexdigest() != entry["sha256"]:
                raise ValueError(f"File verification failed: {entry['path']}")
    return len(names)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "output/releases")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if not output.is_relative_to(ROOT) or output == ROOT:
        parser.error("Output must be a child directory of this project")
    output.mkdir(parents=True, exist_ok=True)
    archive = output / (PACKAGE_NAME + ".zip")
    if archive.exists() or archive.with_suffix(".zip.sha256.txt").exists():
        parser.error("Release already exists; choose another output directory")
    files = collect()
    manifest = {"version": SOURCE_VERSION, "kind": "source-only", "firmware_api": "WHEEL_SPEED_V1",
                "open_loop_api": "MECANUM_V2", "pin_profile": "FOUR_V1", "wheel_order": "CBAD",
                "hardware_acceptance": "pending", "files": [
                    {"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
                    for name, data in sorted(files.items())]}
    files["source-manifest.json"] = encoded(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    with zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED, compresslevel=9) as bundle:
        for name, data in sorted(files.items()):
            bundle.writestr(PACKAGE_NAME + "/" + name, data)
    count = verify_archive(archive)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_suffix(".zip.sha256.txt").write_text(f"{digest}  {archive.name}\n", encoding="utf-8")
    print(json.dumps({"archive": str(archive), "bytes": archive.stat().st_size,
                      "files": count, "sha256": digest}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
