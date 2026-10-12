#!/usr/bin/env python3
"""检查实际 ROS 输入时域和逐点纳秒偏移；保留原始测量时间，不重新盖 now()。"""
import argparse
import json
import math
import struct
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class StampStats:
    def __init__(self):
        self.count = self.backwards = self.duplicates = self.bad_latency = self.bad_points = 0
        self.last = self.first = None
        self.max_gap_ns = 0
        self.min_age_ns = self.max_age_ns = None

    def add(self, stamp, now, max_age):
        self.count += 1
        if self.first is None:
            self.first = stamp
        if self.last is not None:
            difference = stamp - self.last
            self.backwards += difference < 0
            self.duplicates += difference == 0
            self.max_gap_ns = max(self.max_gap_ns, difference)
        self.last = stamp
        age = now - stamp
        self.min_age_ns = age if self.min_age_ns is None else min(self.min_age_ns, age)
        self.max_age_ns = age if self.max_age_ns is None else max(self.max_age_ns, age)
        self.bad_latency += abs(age) > max_age

    def dict(self):
        return vars(self)


def cloud_times(msg):
    field = next((f for f in msg.fields if f.name == 'timestamp'), None)
    if field is None or field.datatype != 8 or field.count != 1 or field.offset + 8 > msg.point_step:
        raise ValueError('MID360 必须有 FLOAT64 timestamp 相对纳秒字段')
    if not msg.width or not msg.height or msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError('点云布局不合法或为空')
    values = []
    fmt = '>d' if msg.is_bigendian else '<d'
    for row in range(msg.height):
        for col in range(msg.width):
            value = struct.unpack_from(fmt, msg.data, row * msg.row_step + col * msg.point_step + field.offset)[0]
            if not math.isfinite(value) or not 0 <= value <= 1e9:
                raise ValueError('逐点偏移不是 0～1 秒范围内的相对纳秒，检查是否误传绝对时间')
            values.append(value)
    if any(b < a for a, b in zip(values, values[1:])):
        raise ValueError('单帧逐点时间发生回退')
    return round(min(values)), round(max(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--max-age-ms', type=float, default=500)
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.seconds, args.max_age_ms)):
        parser.error('采样时间和延迟阈值必须是有限正数')
    rclpy.init(args=[])
    node = rclpy.create_node('ptp_timestamp_monitor')
    streams = {'lidar': StampStats(), 'imu': StampStats()}
    point_range = [None, None]

    def receive(name, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        streams[name].add(stamp, time.time_ns(), round(args.max_age_ms * 1e6))
        if name == 'lidar':
            try:
                first, last = cloud_times(msg)
                if point_range[0] is None:
                    point_range[0] = stamp + first
                point_range[1] = stamp + last
            except (ValueError, struct.error):
                streams[name].bad_points += 1

    node.create_subscription(PointCloud2, args.lidar_topic, lambda m: receive('lidar', m), qos_profile_sensor_data)
    node.create_subscription(Imu, args.imu_topic, lambda m: receive('imu', m), qos_profile_sensor_data)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    failures = []
    for name, stats in streams.items():
        if stats.count < 5:
            failures.append(f'{name}: 消息不足 5 条')
        if stats.backwards or stats.duplicates or stats.bad_latency or stats.bad_points:
            failures.append(f'{name}: 检出回退/重复/UTC偏差/逐点时间异常')
    imu = streams['imu']
    if point_range[0] is None or imu.first is None or max(point_range[0], imu.first) > min(point_range[1], imu.last):
        failures.append('激光逐点时间区间与 IMU 时间区间不重叠')
    report = {'streams': {name: stats.dict() for name, stats in streams.items()},
              'point_time_range_ns': point_range, 'failures': failures, 'passed': not failures,
              'note': '仅测 ROS 时序；接收时间戳回退到主机 now() 也可能通过，必须同时验收原始 time_type=1'}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
