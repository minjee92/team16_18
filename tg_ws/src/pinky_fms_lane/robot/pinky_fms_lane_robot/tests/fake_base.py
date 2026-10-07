#!/usr/bin/env python3
"""시험용 가짜 로봇 (위치 추정 launch 를 실물 없이 시험할 때만 쓴다. 설치하지 않는다).

  /<ns>/tf   : odom → base_footprint (제자리, 20 Hz)
  /<ns>/scan : 지도를 광선 추적해 만든 가짜 라이다. 참 위치에서 본 360 빔, 10 Hz, frame = base_footprint
AMCL 은 참 위치를 모른다. 초기 위치를 받고 이 스캔을 지도와 맞춰서 참 위치를 찾아야 한다.
로봇이 odom 원점에 서 있으므로, AMCL 이 맞게 찾으면 map→odom 이 곧 참 위치가 된다.

    python3 fake_base.py --map <지도.yaml> --pose x,y,yaw [--namespace amr_01] [--frame-prefix amr_01/]
"""
import argparse
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from pinky_fms_traffic.gridmap import read_pgm      # 팀원 코드 재사용 (수정 없음)

BEAMS, RANGE_MIN, RANGE_MAX, STEP = 360, 0.05, 3.5, 0.005


def load_occupancy(map_yaml):
    """지도 → (점유 격자 bool, 해상도, 원점). 행 0 = 월드 y 최소."""
    with open(map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(os.path.dirname(os.path.abspath(map_yaml)), meta['image']))[::-1]
    p = (255 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    return p > meta['occupied_thresh'], float(meta['resolution']), meta['origin'][:2]


def simulate_scan(occ, res, origin, x, y, yaw):
    """(x, y, yaw) 에서 본 라이다 거리. 벽에 닿지 않으면 inf."""
    ang = -math.pi + np.arange(BEAMS) * (2 * math.pi / BEAMS)
    r = np.arange(RANGE_MIN, RANGE_MAX, STEP)
    px = x + np.cos(yaw + ang)[:, None] * r[None, :]
    py = y + np.sin(yaw + ang)[:, None] * r[None, :]
    col = np.floor((px - origin[0]) / res).astype(int)
    row = np.floor((py - origin[1]) / res).astype(int)
    inside = (col >= 0) & (col < occ.shape[1]) & (row >= 0) & (row < occ.shape[0])
    hit = np.zeros(inside.shape, bool)
    hit[inside] = occ[row[inside], col[inside]]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    return np.where(first >= 0, r[np.clip(first, 0, None)], np.inf).astype(np.float32)


class FakeBase(Node):
    def __init__(self, args):
        super().__init__('fake_base', namespace=args.namespace)
        x, y, yaw = (float(v) for v in args.pose.split(','))
        occ, res, origin = load_occupancy(args.map)
        self.ranges = simulate_scan(occ, res, origin, x, y, yaw).tolist()
        self.odom, self.base = args.frame_prefix + 'odom', args.frame_prefix + 'base_footprint'
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 100)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.create_timer(0.05, self.publish_tf)
        self.create_timer(0.1, self.publish_scan)
        hits = sum(math.isfinite(v) for v in self.ranges)
        self.get_logger().info(f'fake_base ready: 참 위치 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°), '
                               f'빔 {BEAMS}개 중 벽에 닿은 빔 {hits}개, 프레임 {self.odom} → {self.base}')

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = self.odom, self.base
        t.transform.rotation.w = 1.0
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def publish_scan(self):
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = self.base
        s.angle_increment = 2 * math.pi / BEAMS
        s.angle_min = -math.pi
        s.angle_max = -math.pi + (BEAMS - 1) * s.angle_increment
        s.scan_time, s.range_min, s.range_max = 0.1, RANGE_MIN, RANGE_MAX
        s.ranges = self.ranges
        self.scan_pub.publish(s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--pose', required=True, help='참 위치 x,y,yaw (map 좌표)')
    ap.add_argument('--namespace', default='amr_01')
    ap.add_argument('--frame-prefix', default='')
    args = ap.parse_args()
    rclpy.init()
    node = FakeBase(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
