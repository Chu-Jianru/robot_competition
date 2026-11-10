"""离线回归入口；可在项目或解压后的源码包中运行，不打开串口或真实相机。"""
from pathlib import Path
import argparse
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE if (HERE / "motor-bringup").is_dir() else HERE.parent


def run(command):
    print("+ " + subprocess.list2cmdline([str(part) for part in command]), flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cpp", action="store_true", help="同时编译运行 8 组 C++ 测试，需要 g++")
    parser.add_argument("--firmware", action="store_true", help="同时运行 PlatformIO 编译，不烧录")
    args = parser.parse_args()
    try:
        for folder in ("motor-bringup/tests", "debug-workbench/tests", "esp32-pc-link/pc/tests"):
            run([sys.executable, "-m", "unittest", "discover", "-s", folder, "-p", "test_*.py"])
        if args.cpp:
            compiler = shutil.which("g++")
            if compiler is None:
                raise RuntimeError("g++ not found; install it or run without --cpp.")
            build = ROOT / "tmp/offline-checks"
            build.mkdir(parents=True, exist_ok=True)
            suites = ("test_wheel_speed", "test_calibration", "test_config_store", "test_closed_loop",
                      "test_motor", "test_chassis", "test_bluetooth")
            for suite in suites:
                binary = build / (suite + (".exe" if sys.platform == "win32" else ""))
                run([compiler, "-std=c++17", "-I", "motor-bringup/tests",
                     f"motor-bringup/tests/{suite}.cpp", "motor-bringup/src/dry_core.cpp", "-o", binary])
                run([binary])
            core_binary = build / ("test_link_core.exe" if sys.platform == "win32" else "test_link_core")
            run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", "esp32-pc-link/lib/link_core",
                 "esp32-pc-link/host-tests/test_link_core.cpp", "esp32-pc-link/lib/link_core/link_core.cpp",
                 "-o", core_binary])
            run([core_binary])
        if args.firmware:
            platformio = shutil.which("pio")
            if platformio is None:
                candidate = Path.home() / ".platformio/penv/Scripts/pio.exe"
                platformio = str(candidate) if candidate.is_file() else None
            if platformio is None:
                raise RuntimeError("PlatformIO not found; install it or run without --firmware.")
            run([platformio, "run", "--project-dir", "motor-bringup"])
    except (subprocess.CalledProcessError, RuntimeError) as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 1
    print("PASS: all requested offline checks completed; no hardware accessed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
