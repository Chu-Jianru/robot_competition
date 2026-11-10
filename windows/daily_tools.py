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
CAPTURES = ROOT/'windows/captures_hi'
ACTIONS = {
    '1': ('capture', '看画面、拍照片'),
    '2': ('tag', '看 tag 和台架红块检测'),
    '3': ('stats', '统计 60 帧 tag 检出情况'),
    '4': ('calibrate', '标定场地四角'),
    '5': ('pose', '按场地标定查看车辆位置'),
    '6': ('results', '查看已有识别结果'),
    '7': ('label', '标注照片'),
    '8': ('usb', '检查手机 USB 连接'),
    '9': ('panel', '打开底盘工作台'),
    '10': ('task', '运行自主清扫任务'),
    '11': ('task-config', '保存自主任务的本车运动参数'),
    '12': ('task-preview', '预览自主清扫策略（不接底盘）'),
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


def choose_batch():
    print('\n1  无车照片（第二批）\n2  带车照片（第一批）\n0  返回')
    value = input('请选择：').strip()
    return {'1': 'no_robot', '2': 'with_robot'}.get(value)


def command(action, *, batch='no_robot', image=None):
    python = sys.executable
    camera = ['--scrcpy-dir', str(SCRCPY), '--rotate', '90', '--max-size', '1280']
    if action in ('task', 'task-config', 'task-preview'):
        args = [python, str(ROOT/'windows/run_task.py'), *camera]
        return args + {'task': [], 'task-config': ['--configure'], 'task-preview': ['--preview']}[action]
    if action in ('capture', 'tag', 'pose'):
        args = [python, str(ROOT/'windows/capture_hi.py'), *camera, '--no-fallback']
        if action == 'tag':
            args += ['--bench', '--detect-hz', '10']
        if action == 'pose':
            args += ['--config', str(CALIB), '--detect-hz', '10']
        return args
    if action == 'stats':
        return [python, str(ROOT/'windows/live_bench.py'), *camera, '--frames', '60']
    if action == 'calibrate':
        return [python, str(ROOT/'windows/field_view.py'), 'calibrate',
                str(image or '<选择的原图>'), '--out', str(CALIB)]
    if action == 'label':
        folder = 'field_no_robot_20261103' if batch == 'no_robot' else 'field_photos_20261103'
        return [python, str(ROOT/'windows/analyze_field_photos.py'),
                '--input', str(ROOT/'windows/test_data'/folder), '--annotate']
    if action == 'usb':
        return [str(SCRCPY/'adb.exe'), 'devices', '-l']
    if action == 'panel':
        return ['cmd.exe', '/d', '/c', str(ROOT/'tools/启动新面板.cmd')]
    if action == 'results':
        folder = ('field_no_robot_20261103/optimized_v2' if batch == 'no_robot'
                  else 'field_photos_20261103/regression_v2')
        return [str(ROOT/'log'/folder/'detections_contact.jpg')]
    raise ValueError(action)


def launch(action, *, batch=None, dry_run=False, phone_checks=False):
    if dry_run:
        print(json.dumps(command(action, batch=batch or 'no_robot'), ensure_ascii=False))
        return 0
    if action == 'usb':
        return 0 if check_phone() else 1
    if action in ('pose', 'task-preview', 'task') and not CALIB.is_file():
        print('先用 1 拍全场原图，再用 4 点四角。')
        return 1
    if action == 'task' and not TASK_CONFIG.is_file():
        print('先用 11 保存实测底盘参数。')
        return 1
    if phone_checks and action in ('capture', 'tag', 'stats', 'pose', 'task-preview', 'task'):
        serial = check_phone()
        if serial is None or not camera_available(serial):
            return 1
    if action == 'task':
        print('给底盘上电、释放调试面板占用的串口，把车放在蓝区，车头朝黄区。')
        print('任务窗口显示 SPACE to start 时按空格。Q 或 Esc 停车退出。')
    if action in ('results', 'label') and batch is None:
        batch = choose_batch()
        if batch is None:
            return 130
    image = None
    if action == 'calibrate':
        import tkinter as tk
        from tkinter import filedialog
        print('选择正式取流保存的原图，不要选 marked/annotated 标注图。')
        print('依次点：画面左上 → 右上 → 右下 → 左下的地面内角。右键撤销，回车保存。')
        print('本次标定将替换 windows/field_calib.json；机位、分辨率和旋转须与取流一致。')
        picker = tk.Tk()
        picker.withdraw()
        try:
            image = filedialog.askopenfilename(
                parent=picker, title='选择完整场地原图',
                initialdir=str(CAPTURES if CAPTURES.exists() else ROOT),
                filetypes=[('图片', '*.png *.jpg *.jpeg'), ('所有文件', '*.*')])
        finally:
            picker.destroy()
        if not image:
            return 130
    args = command(action, batch=batch or 'no_robot', image=image)
    if action == 'results':
        if Path(args[0]).exists():
            os.startfile(args[0])
            print(f'已打开 {args[0]}')
            return 0
        print(f'尚无结果图片：{args[0]}')
        return 1
    if action in ('capture', 'tag', 'stats', 'pose'):
        print('请先关闭其他相机客户端；本工具不发送电机指令。')
    if action in ('capture', 'tag', 'pose'):
        print('窗口快捷键：S 保存，R 旋转，Q/Esc 退出。')
    if action in ('tag', 'stats'):
        print('此处检查 tag 和台架红块；整场策略识别请选 12。')
    if action == 'pose':
        print('此处查看场地校正和车体位姿；整场策略识别请选 12。')
    if action == 'label':
        print('B 点红块，K 点四角，右键撤销，N 下一张，Q 保存退出。')
        if batch == 'no_robot':
            print('无车 01 缺真实地面角，不猜四角。')
    if action == 'panel':
        print('将打开底盘工作台。请核对本车实物轮位，连接和运动由你在面板中操作。')
    sys.stdout.flush()
    status = subprocess.run(args, cwd=ROOT).returncode
    if status == 0 and action == 'calibrate':
        status = subprocess.run([sys.executable, str(ROOT/'windows/field_view.py'), 'preview',
                                 image, '--config', str(CALIB)], cwd=ROOT).returncode
        if status:
            print(f'四角已保存到 {CALIB}，校正图预览未完成。')
    if status not in (0, 130):
        print(f'工具返回码 {status}，请查看上方报错。')
    return status


def run_menu_action(choice, *, batch=None, phone_checks=False):
    action, title = ACTIONS[choice]
    rule = '=' * 54
    print('\n' + menu_text(f'{rule}\n【开始】{choice}  {title}\n{rule}', 36), flush=True)
    try:
        status = launch(action, batch=batch, phone_checks=phone_checks)
    except (EOFError, KeyboardInterrupt):
        status = 130
    except Exception as error:
        print(f'操作出错：{error}')
        status = 1
    label, color = {0: ('已结束', 32), 130: ('已取消', 33)}.get(status, ('未完成', 31))
    print('\n' + menu_text(f'{rule}\n【{label}】{choice}  {title}', color), flush=True)
    if status == 0 and action == 'stats':
        print('请以上方「判定」一行为准。')
    print(menu_text(rule, color), flush=True)
    return status


def show_menu():
    print('\n' + menu_text('清扫机器人 · 日常工具', 36))
    for number, (_, title) in ACTIONS.items():
        print(f'{number}  {title}')
    print('0  退出')


def adb_output(*args):
    return subprocess.check_output(
        [str(SCRCPY/'adb.exe'), *args], encoding='utf-8', errors='replace',
        timeout=12, cwd=ROOT,
    )


def check_phone():
    deadline = time.monotonic() + 5
    while True:
        output = adb_output('devices', '-l')
        _, _, rows = output.partition('List of devices attached')
        devices = [row.split() for row in rows.splitlines() if len(row.split()) >= 2]
        usb = [row for row in devices if ':' not in row[0] and not row[0].startswith('emulator-')]
        if usb or time.monotonic() >= deadline:
            break
        time.sleep(.4)
    print(output.strip())
    ready = [row for row in usb if row[1] == 'device']
    if len(ready) == 1:
        print('USB 通过，已找到一台已授权手机。取景和连续取流请用步骤 1 确认。')
        return ready[0][0]
    if len(ready) > 1:
        print('检测到多台已授权 USB 手机，请只保留用于俯视的手机。')
    elif any(row[1] == 'unauthorized' for row in usb):
        print('请解锁手机，在手机上点允许 USB 调试，再重试。')
    elif any(row[1] == 'offline' for row in usb):
        print('手机处于 offline，请拔插数据线，等手机恢复连接后再重试。')
    else:
        print('未检测到已授权 USB 手机。请确认开启 USB 调试并使用能传数据的线。')
    return None


def camera_available(serial):
    forwards = adb_output('forward', '--list').splitlines()
    reverses = adb_output('-s', serial, 'reverse', '--list').splitlines()
    active = [line for line in forwards if line.split()[:1] == [serial]] + reverses
    if any('localabstract:scrcpy' in line for line in active):
        print('手机已有 scrcpy 通道。请退出正在取图的窗口，然后重试。')
        return False
    return True


def phone_flow(check_only):
    status = run_menu_action('8')
    if check_only:
        return status
    choices = ('8', '1', '2', '3', '4', '5', '9', '11', '12', '10')
    while True:
        print('\n' + menu_text('手机接入流程', 36))
        print(f"场地标定 {'已有' if CALIB.is_file() else '待完成'}，本车参数 {'已有' if TASK_CONFIG.is_file() else '待完成'}")
        print('单调手机用 8 → 1 → 2，可选 3 统计。')
        print('首次跑任务用 1 → 4 → 11 → 12 → 10。以后机位和参数相同时，检查画面后可直接选 10。')
        for choice in choices:
            print(choice, ACTIONS[choice][1])
        print('0  退出')
        try:
            choice = input('选择步骤：').strip()
            if choice == '0':
                return 0
            if choice not in choices:
                print('请输入上面列出的编号。')
                continue
            run_menu_action(choice, phone_checks=True)
        except (EOFError, KeyboardInterrupt):
            return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--action', choices=[a[0] for a in ACTIONS.values()])
    modes.add_argument('--phone-flow', action='store_true', help='检查 USB 并显示接入流程')
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
    if args.phone_flow or args.check_phone:
        return phone_flow(args.check_phone)
    if args.action:
        if args.dry_run:
            return launch(args.action, batch=args.batch, dry_run=True)
        choice = next(number for number, (action, _) in ACTIONS.items() if action == args.action)
        return run_menu_action(choice, batch=args.batch)
    while True:
        show_menu()
        try:
            choice = input('输入数字并回车：').strip()
            if choice == '0':
                return
            if choice not in ACTIONS:
                print('请输入菜单中的数字。')
                continue
            run_menu_action(choice)
        except (EOFError, KeyboardInterrupt):
            return


if __name__ == '__main__':
    raise SystemExit(main())
