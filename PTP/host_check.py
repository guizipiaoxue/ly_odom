#!/usr/bin/env python3
"""只读检查主机 PTP、网络与当前驱动接收地址，不把 MASTER 当作雷达已锁定。"""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess


ROOT = Path(__file__).resolve().parent.parent


def command(*args):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def interface_for(address):
    routes = json.loads(command('ip', '-j', 'route', 'get', address) or '[]')
    if not routes or 'dev' not in routes[0]:
        raise RuntimeError(f'未找到到 {address} 的路由，请显式指定有线网卡')
    return routes[0]['dev']


def clock_device(interface):
    clocks = list((Path('/sys/class/net') / interface / 'device/ptp').glob('ptp[0-9]*'))
    if len(clocks) != 1:
        raise RuntimeError(f'{interface} 没有唯一的硬件 PHC，不能使用本硬件 PTP 配置')
    return '/dev/' + clocks[0].name


def processes(program):
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
            if args and Path(args[0]).name == program:
                result.append({'pid': int(entry.name), 'args': args})
        except (OSError, UnicodeError):
            continue
    return result


def collect(interface, lidar_ip):
    issues = []
    addresses = json.loads(command('ip', '-j', 'addr', 'show', 'dev', interface) or '[]')
    ips = [a['local'] for item in addresses for a in item.get('addr_info', []) if a['family'] == 'inet']
    try:
        phc = clock_device(interface)
    except RuntimeError as error:
        phc = None
        issues.append(str(error))
    timestamps = command('ethtool', '-T', interface) if shutil.which('ethtool') else '未安装 ethtool'
    if not shutil.which('ethtool'):
        issues.append('安装脚本会补装 ethtool，以确认收发硬件时间戳能力')
    elif not all(flag in timestamps for flag in ('hardware-transmit', 'hardware-receive', 'hardware-raw-clock')):
        issues.append('ethtool 未确认完整硬件时间戳能力')
    config = ROOT / 'livox_sdk/src/livox_ros_driver2/config/MID360_config.json'
    host_ips = []
    if config.exists():
        host = json.loads(config.read_text())['MID360']['host_net_info']
        host_ips = sorted({value for key, value in host.items() if key.endswith('_ip') and value})
        for value in host_ips:
            if value not in ips:
                issues.append(f'驱动接收 IP {value} 未配置到 {interface}；雷达数据可能无法送达')
    ptp = processes('ptp4l')
    phc2sys = processes('phc2sys')
    if not ptp or not phc2sys:
        issues.append('未同时发现 ptp4l 和 phc2sys，需确认对应网卡实例')
    result = {
        'interface': interface, 'lidar_ip': lidar_ip, 'ipv4': ips, 'phc': phc,
        'driver_receiver_ips': host_ips, 'ethtool': timestamps,
        'linuxptp_version': command('ptp4l', '-v'),
        'ptp4l_processes': ptp, 'phc2sys_processes': phc2sys,
        'time_status': command('timedatectl', 'show', '-p', 'NTPSynchronized', '-p', 'NTP', '-p', 'Timezone'),
        'issues': issues,
        'lidar_ptp_verified': False,
        'note': '主机服务和 PHC 状态不能证明雷达锁定；需检查 UDP time_type=1 和 ROS 时间戳',
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface')
    parser.add_argument('--lidar-ip', default='192.168.1.3')
    parser.add_argument('--interface-only', action='store_true')
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--require-network', action='store_true',
                        help='只检查驱动接收地址是否在指定网卡，用于启动前门控')
    args = parser.parse_args()
    interface = args.interface or interface_for(args.lidar_ip)
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', interface):
        parser.error('网卡名称格式无效')
    if args.interface_only:
        print(interface)
        return 0
    report = collect(interface, args.lidar_ip)
    if args.require_network:
        missing = set(report['driver_receiver_ips']) - set(report['ipv4'])
        if missing:
            print(f'驱动接收地址 {sorted(missing)} 未配置到 {interface}', flush=True)
            return 1
        if not report['driver_receiver_ips']:
            print('未找到源码驱动接收地址配置，不能验收网络')
            return 1
        return 0
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(f'雷达网卡：{interface}；地址：{report["ipv4"]}；PHC：{report["phc"]}')
        print(f'linuxptp：{report["linuxptp_version"]}；驱动接收 IP：{report["driver_receiver_ips"]}')
        print(report['ethtool'])
        for key in ('ptp4l_processes', 'phc2sys_processes'):
            for item in report[key]:
                print(f'PID {item["pid"]}: ' + ' '.join(item['args']))
        print(report['time_status'])
        for issue in report['issues']:
            print('待处理：' + issue)
        print(report['note'])
    return 1 if report['issues'] else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as error:
        raise SystemExit(str(error))
