"""中文日常菜单。只有选中操作后才启动对应工具；默认不连接设备。"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent.parent
SCRCPY = Path(r'E:\scrcpy\scrcpy-win64-v5.0.1')
CALIB = ROOT/'windows/field_calib.json'
TASK_CONFIG = ROOT/'windows/task_drive.json'
ACTIONS = {
    '1': ('capture', '实时看图、拍照和场地标定'),
    '2': ('tag', '看 tag 和台架红块检测'),
    '5': ('pose', '按场地标定查看车辆位置'),
    '6': ('results', '查看已有识别结果'),
    '7': ('label', '标注照片'),
    '8': ('usb', '检查手机 USB 连接'),
    '9': ('panel', '连接底盘与轮速标定'),
    '10': ('task', '运行自主清扫任务'),
    '11': ('task-config', '用相机和四轮编码器自动标定（车会动）'),
    '12': ('task-preview', '预览自主清扫策略（不接底盘）'),
    '13': ('keyboard', '键盘遥控小车'),
}


def enable_menu_color():
    if os.environ.get('ROBOT_MENU_COLOR') != '1' or not sys.stdout.isatty():
        return False
    if os.name != 'nt':
        return True

    import atexit
    import ctypes
    import msvcrt
    from ctypes import wintypes

    kernel = ctypes.windll.kernel32
    handle = wintypes.HANDLE(msvcrt.get_osfhandle(sys.stdout.fileno()))
    mode = wintypes.DWORD()
    if not kernel.GetConsoleMode(handle, ctypes.byref(mode)):
        return False
    vt_mode = mode.value | 0x0005  # Process output and ANSI control sequences.
    if mode.value == vt_mode:
        return True
    if not kernel.SetConsoleMode(handle, vt_mode):
        return False
    atexit.register(kernel.SetConsoleMode, handle, mode.value)
    return True


MENU_COLOR = enable_menu_color()


def menu_text(text, color):
    return f'\x1b[1;{color}m{text}\x1b[0m' if MENU_COLOR else text


def has_encoder_calibration():
    try:
        saved = json.loads(TASK_CONFIG.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return False
    return isinstance(saved, dict) and 'forward_counts_per_mm' in saved


def choose_batch():
    print('\n1  无车照片（第二批）\n2  带车照片（第一批）\n0  返回')
    value = input('请选择：').strip()
    return {'1': 'no_robot', '2': 'with_robot'}.get(value)


def command(action, *, batch='no_robot'):
    python = sys.executable
    camera = ['--scrcpy-dir', str(SCRCPY), '--rotate', '90', '--max-size', '1280']
    if action in ('task', 'task-config', 'task-preview'):
        args = [python, str(ROOT/'windows/run_task.py'), *camera]
        return args + {'task': [], 'task-config': ['--configure'], 'task-preview': ['--preview']}[action]
    if action in ('capture', 'tag', 'pose'):
        args = [python, str(ROOT/'windows/capture_hi.py'), *camera, '--no-fallback']
        if action == 'capture':
            args += ['--calibrate-out', str(CALIB)]
        if action == 'tag':
            args += ['--bench', '--detect-hz', '10']
        if action == 'pose':
            args += ['--config', str(CALIB), '--detect-hz', '10']
        return args
    if action == 'label':
        folder = 'field_no_robot_20261103' if batch == 'no_robot' else 'field_photos_20261103'
        return [python, str(ROOT/'windows/analyze_field_photos.py'),
                '--input', str(ROOT/'windows/test_data'/folder), '--annotate']
    if action == 'usb':
        return [str(SCRCPY/'adb.exe'), 'devices', '-l']
    if action == 'panel':
        return [python, str(ROOT/'hardware/firmware/WHEEL_SPEED_V1/motor-bringup/wheel_calibration_panel.py')]
    if action == 'keyboard':
        return [python, '-u', str(ROOT/'windows/keyboard_drive.py')]
    if action == 'results':
        folder = ('field_no_robot_20261103/optimized_v2' if batch == 'no_robot'
                  else 'field_photos_20261103/regression_v2')
        return [str(ROOT/'log'/folder/'detections_contact.jpg')]
    raise ValueError(action)


def launch(action, *, batch=None, dry_run=False):
    if dry_run:
        print(json.dumps(command(action, batch=batch or 'no_robot'), ensure_ascii=False))
        return 0
    if action == 'usb':
        return 0 if check_phone(report_success=True) else 1
    if action in ('pose', 'task-preview', 'task', 'task-config') and not CALIB.is_file():
        print(menu_text('缺少场地标定。选 1 按 S 拍全场，点四角后按回车保存。', 31))
        return 1
    if action == 'task' and not has_encoder_calibration():
        print(menu_text('需要本车编码器参数。先选 11 自动标定。', 31))
        return 1
    if action in ('capture', 'tag', 'pose', 'task-preview', 'task', 'task-config'):
        serial = check_phone()
        if serial is None or not camera_available(serial):
            return 1
    if action == 'task':
        print('给底盘上电并关闭调试面板。把车放在己方半区靠外侧，车头朝中线。')
        print('任务窗口显示 SPACE to start 时按空格。Q 或 Esc 停车退出。')
    if action in ('results', 'label') and batch is None:
        batch = choose_batch()
        if batch is None:
            return 130
    args = command(action, batch=batch or 'no_robot')
    if action == 'results':
        if Path(args[0]).exists():
            os.startfile(args[0])
            print(menu_text('已打开识别结果。', 32))
            return 0
        print(menu_text(f'尚无结果图片：{args[0]}', 31))
        return 1
    if action == 'capture':
        print('S 拍照，点四角后回车保存。Q 从照片返回视频，Esc 从视频返回菜单。')
    if action in ('tag', 'pose'):
        print('窗口快捷键：S 保存，R 旋转，Q/Esc 退出。')
    if action == 'label':
        print('B 点红块，K 点四角，右键撤销，N 下一张，Q 保存退出。')
        if batch == 'no_robot':
            print('无车 01 缺真实地面角，不猜四角。')
    if action == 'panel':
        print('将打开轮速标定窗口。选择连接后，确认四轮架空才可开始标定。')
    sys.stdout.flush()
    status = subprocess.run(args, cwd=ROOT).returncode
    if status not in (0, 130):
        print(menu_text(f'工具执行失败（退出码 {status}）。', 31))
    return status


def run_menu_action(choice, *, batch=None):
    action = ACTIONS[choice][0]
    try:
        status = launch(action, batch=batch)
    except (EOFError, KeyboardInterrupt):
        status = 130
    except Exception as error:
        print(menu_text(f'操作出错：{error}', 31))
        return 1
    if status == 0 and action not in ('usb', 'results'):
        print(menu_text('已结束。', 32))
    elif status == 130:
        print('已取消。')
    return status


def show_menu():
    print('\n清扫机器人 · 日常工具')
    print(f"场地标定 {'已有' if CALIB.is_file() else '待完成'}，编码器运动标定 {'已有' if has_encoder_calibration() else '待完成'}")
    for number, (_, title) in ACTIONS.items():
        print(f'{number}  {title}')
    print('0  退出')


def adb_output(*args):
    try:
        return subprocess.check_output(
            [str(SCRCPY/'adb.exe'), *args], encoding='utf-8', errors='replace',
            stderr=subprocess.PIPE, timeout=12, cwd=ROOT,
        )
    except subprocess.CalledProcessError as error:
        detail = ' '.join((error.stderr or '').split())
        raise RuntimeError(detail or f'ADB 执行失败（退出码 {error.returncode}）。') from error
    except subprocess.TimeoutExpired as error:
        raise RuntimeError('ADB 响应超时，请检查手机 USB 连接。') from error


def check_phone(*, report_success=False):
    deadline = time.monotonic() + 5
    while True:
        output = adb_output('devices', '-l')
        _, _, rows = output.partition('List of devices attached')
        devices = [row.split() for row in rows.splitlines() if len(row.split()) >= 2]
        usb = [row for row in devices if ':' not in row[0] and not row[0].startswith('emulator-')]
        if usb or time.monotonic() >= deadline:
            break
        time.sleep(.4)
    ready = [row for row in usb if row[1] == 'device']
    if len(ready) == 1:
        if report_success:
            print(menu_text('USB 通过，已找到一台已授权手机。', 32))
        return ready[0][0]
    if len(ready) > 1:
        message = '检测到多台已授权 USB 手机，请只保留用于俯视的手机。'
    elif any(row[1] == 'unauthorized' for row in usb):
        message = '手机未授权。请解锁手机并允许 USB 调试。'
    elif any(row[1] == 'offline' for row in usb):
        message = '手机离线。请拔插数据线，等手机恢复连接后再重试。'
    else:
        message = '未检测到已授权 USB 手机。请检查数据线和 USB 调试。'
    print(menu_text(message, 31))
    return None


def camera_available(serial):
    forwards = adb_output('forward', '--list').splitlines()
    reverses = adb_output('-s', serial, 'reverse', '--list').splitlines()
    active = [line for line in forwards if line.split()[:1] == [serial]] + reverses
    if any('localabstract:scrcpy' in line for line in active):
        print(menu_text('手机已有 scrcpy 通道。请退出正在取图的窗口，再重试。', 31))
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--action', choices=[a[0] for a in ACTIONS.values()])
    # 兼容已复制到本机的旧 PowerShell 启动脚本。
    modes.add_argument('--phone-flow', action='store_true', help=argparse.SUPPRESS)
    modes.add_argument('--check-phone', action='store_true', help='仅检查 USB 识别与授权')
    parser.add_argument('--batch', choices=['no_robot', 'with_robot'])
    parser.add_argument('--dry-run', action='store_true', help='只打印命令，不启动工具')
    parser.add_argument('--list', action='store_true', help='只显示菜单，不启动工具')
    args = parser.parse_args()
    if args.list:
        show_menu()
        return
    if args.dry_run and not args.action:
        parser.error('--dry-run 需要 --action')
    if args.check_phone:
        return run_menu_action('8')
    if args.action:
        if args.dry_run:
            return launch(args.action, batch=args.batch, dry_run=True)
        choice = next(number for number, (action, _) in ACTIONS.items() if action == args.action)
        return run_menu_action(choice, batch=args.batch)
    if args.phone_flow:
        run_menu_action('8')
    show_menu()
    while True:
        try:
            choice = input('输入编号（直接回车查看菜单）：').strip()
            if choice == '0':
                return 0
            if not choice:
                show_menu()
                continue
            if choice not in ACTIONS:
                print(menu_text('请输入菜单中的编号。', 31))
                continue
            run_menu_action(choice)
        except (EOFError, KeyboardInterrupt):
            return 0


if __name__ == '__main__':
    raise SystemExit(main())
