#!/usr/bin/env python3
"""仅在采集前初始化网卡 PHC；绝不设置主机 CLOCK_REALTIME。"""
import os
from pathlib import Path
import sys
import time


def clock_device(interface):
    clocks = list((Path('/sys/class/net') / interface / 'device/ptp').glob('ptp[0-9]*'))
    if len(clocks) != 1:
        raise RuntimeError(f'{interface} 没有唯一的硬件 PHC，不能使用本硬件 PTP 配置')
    return '/dev/' + clocks[0].name


def processes(program):
    # 初始化前只读确认没有正在运行的授时进程。
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
            if args and Path(args[0]).name == program:
                return True
        except (OSError, UnicodeError):
            continue
    return False


def bootstrap(interface, config):
    # 配置固定为 UTC→PHC 的主时钟模式，PHC 时间为 UTC + TAI-UTC。
    settings = {}
    for raw in Path(config).read_text().splitlines():
        parts = raw.split('#', 1)[0].split()
        if len(parts) == 2:
            settings[parts[0]] = parts[1]
    if settings.get('masterOnly') != '1' or settings.get('time_stamping') != 'hardware':
        raise RuntimeError('PHC 初始化仅支持固定 hardware MASTER 配置')
    offset = int(settings['utc_offset'])
    if not 0 <= offset <= 255:
        raise RuntimeError('utc_offset 无效')
    deadline = time.monotonic() + 30
    while True:
        try:
            if (Path('/sys/class/net') / interface / 'carrier').read_text().strip() == '1':
                device = clock_device(interface)
                break
        except OSError:
            pass
        if time.monotonic() >= deadline:
            raise RuntimeError('网卡或链路未就绪')
        time.sleep(0.2)
    with open(device, 'rb', buffering=0) as phc:
        clock_id = ((~phc.fileno()) << 3) | 3  # Linux FD_TO_CLOCKID。
        before = time.clock_gettime_ns(clock_id)
        target = time.time_ns() + offset * 10**9
        difference = target - before
        print(f'{device}: 相对 UTC+{offset} 秒偏差 {difference} ns', flush=True)
        if abs(difference) <= 1_000_000:
            return
        # 已有伺服器或采集进程时禁止阶跃，否则正在发布的传感器时间可能回退。
        if processes('ptp4l') or processes('phc2sys'):
            raise RuntimeError('已有 PTP 进程，禁止同时初始化 PHC')
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                args = entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
            except FileNotFoundError:
                continue
            if not args:
                continue
            names = [Path(arg).name for arg in args[:3]]
            if any(name in ('livox_ros_driver2_node', 'ly_lio_node', 'point_lio', 'rosbag2_recorder', 'rosbag2_player')
                   for name in names) or ('ros2' in names and 'bag' in args):
                raise RuntimeError(f'发现采集/里程计 PID {entry.name}，禁止 PHC 阶跃；先停止采集再初始化')
        time.clock_settime_ns(clock_id, time.time_ns() + offset * 10**9)
        print('PHC 已初始化；系统时间未修改。', flush=True)


if __name__ == '__main__':
    try:
        bootstrap(*sys.argv[1:])
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(str(error))
