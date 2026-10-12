#!/usr/bin/env python3
"""旁路验收 MID360 原始包：PTP 标志、时序、点云/IMU 时域和 UTC/TAI 偏差。"""
import argparse
from collections import Counter
import json
import math
import socket
import struct
import time


def decode_packet(frame, lidar_ip):
    """解析 Ethernet / VLAN / IPv4 / UDP；不绑定雷达接收 UDP 端口。"""
    if len(frame) < 14:
        return None
    kind = struct.unpack_from('!H', frame, 12)[0]
    offset = 14
    while kind in (0x8100, 0x88A8):
        if len(frame) < offset + 4:
            return None
        kind = struct.unpack_from('!H', frame, offset + 2)[0]
        offset += 4
    if kind != 0x0800 or len(frame) < offset + 20:
        return None
    ihl = (frame[offset] & 15) * 4
    if frame[offset] >> 4 != 4 or ihl < 20 or frame[offset + 9] != 17:
        return None
    if socket.inet_ntoa(frame[offset + 12:offset + 16]) != lidar_ip:
        return None
    if struct.unpack_from('!H', frame, offset + 6)[0] & 0x3FFF:
        return None  # 不将 IPv4 分片误读为完整传感器包。
    total = struct.unpack_from('!H', frame, offset + 2)[0]
    udp = offset + ihl
    if total < ihl + 8 or len(frame) < offset + total:
        return None
    source, destination, length = struct.unpack_from('!HHH', frame, udp)
    if source not in (56300, 56400) or length < 8 or udp + length > offset + total:
        return None
    payload = frame[udp + 8:udp + length]
    # Livox-SDK2 Ethernet header 为 36 字节，time_type 在 11、timestamp 在 28。
    if len(payload) < 36 or struct.unpack_from('<H', payload, 1)[0] != len(payload):
        return None
    data_type = payload[10]
    if source == 56400 and data_type != 0:
        return None
    if source == 56300 and data_type not in (1, 2, 3):
        return None
    return ('imu' if data_type == 0 else 'lidar', payload[11],
            struct.unpack_from('<Q', payload, 28)[0], destination)


class PacketStats:
    def __init__(self, utc_offset_seconds=0):
        self.utc_offset_ns = utc_offset_seconds * 10**9
        self.streams = {name: {'count': 0, 'types': Counter(), 'last': None,
                              'backwards': 0, 'duplicates': 0, 'max_gap_ns': 0,
                              'latency_min_ns': None, 'latency_max_ns': None,
                              'ports': Counter()} for name in ('lidar', 'imu')}

    def add(self, packet, received_ns):
        name, kind, stamp, port = packet
        if kind == 1:
            stamp -= self.utc_offset_ns  # 原始 PTP/TAI 转为与主机相同的 UTC 时域。
        s = self.streams[name]
        s['count'] += 1
        s['types'][kind] += 1
        s['ports'][port] += 1
        if s['last'] is not None:
            difference = stamp - s['last']
            s['backwards'] += difference < 0
            s['duplicates'] += difference == 0
            s['max_gap_ns'] = max(s['max_gap_ns'], difference)
        s['last'] = stamp
        latency = received_ns - stamp
        s['latency_min_ns'] = latency if s['latency_min_ns'] is None else min(s['latency_min_ns'], latency)
        s['latency_max_ns'] = latency if s['latency_max_ns'] is None else max(s['latency_max_ns'], latency)

    def report(self, max_latency_ns):
        failures = []
        for name, s in self.streams.items():
            if s['count'] < 10:
                failures.append(f'{name}: 不足 10 包，无法验收')
                continue
            if set(s['types']) != {1}:
                failures.append(f'{name}: time_type 必须始终为 PTP=1，实测 {dict(s["types"])}')
            if s['backwards'] or s['duplicates']:
                failures.append(f'{name}: 时间回退 {s["backwards"]}，重复 {s["duplicates"]}')
            if max(abs(s['latency_min_ns']), abs(s['latency_max_ns'])) > max_latency_ns:
                failures.append(f'{name}: 设备时间与系统 UTC 相差超过阈值；检查 37 秒 TAI/UTC 或锁定状态')
        return {'streams': self.streams, 'failures': failures, 'passed': not failures,
                'ptp_utc_offset_seconds': self.utc_offset_ns // 10**9,
                'note': '包接收延迟包含网络/调度延迟；PTP=1 且 UTC 一致是验收条件，不是精确授时误差测量'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--lidar-ip', default='192.168.1.3')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--max-latency-ms', type=float, default=100)
    parser.add_argument('--ptp-utc-offset', type=int, default=37,
                        help='实测设备原始 PTP 为 TAI；UTC 固件显式设 0')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.seconds, args.max_latency_ms)):
        parser.error('采样时间与延迟阈值必须为有限正数')
    socket.inet_aton(args.lidar_ip)
    if not 0 <= args.ptp_utc_offset <= 255:
        parser.error('PTP 时差必须为 0..255 整数秒')
    stats = PacketStats(args.ptp_utc_offset)
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003)) as capture:
        capture.bind((args.interface, 0))
        capture.settimeout(0.2)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            try:
                frame = capture.recv(65535)
            except socket.timeout:
                continue
            packet = decode_packet(frame, args.lidar_ip)
            if packet:
                stats.add(packet, time.time_ns())
    report = stats.report(round(args.max_latency_ms * 1e6))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except PermissionError:
        raise SystemExit('AF_PACKET 旁路抓包需要 sudo；不会占用驱动 UDP 接收端口。')
