#!/usr/bin/env python3
"""미션이 FAILED 로 끝날 때, 직전 10초 동안 로봇 주변 지역 코스트맵을 저장한다 (충돌 판정이 어느 칸 때문에 났는지 보기 위함).

  python3 record_costmap.py [출력폴더]      (fms_env.sh 적용한 터미널, 기본 ~/pinky/fms_logs/costmap)
저장: <미션ID>_<로봇>.json  — 1초마다 {t, 로봇 odom 위치, footprint, 로봇 주변 1.0x1.0 m 의 코스트맵 값}.
값: 100=lethal(장애물 칸), 99=inscribed, 1~98=inflation 비용, 0=무비용, -1=미확인  (OccupancyGrid 변환값)
"""
import json
import math
import os
import re
import sys
import time
from collections import defaultdict, deque

import rclpy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

from pinky_fms_interfaces.msg import TaskState

CM = re.compile(r'^/([a-z][a-z0-9_]*)/local_costmap/costmap$')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self, out):
        super().__init__('costmap_recorder')
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.subs = {}
        self.odom = {}
        self.fp = {}
        self.buf = defaultdict(lambda: deque(maxlen=12))
        self.last = {}
        self.create_subscription(TaskState, '/fleet/task_state', self.on_task, 100)
        self.create_timer(3.0, self.discover)

    def discover(self):
        for name, _ in self.get_topic_names_and_types():
            m = CM.match(name)
            if m and m.group(1) not in self.subs:
                r = m.group(1)
                self.subs[r] = [
                    self.create_subscription(OccupancyGrid, name, lambda g, r=r: self.on_map(r, g), 1),
                    self.create_subscription(Odometry, f'/{r}/odom', lambda o, r=r: self.odom.__setitem__(r, o), 1),
                    self.create_subscription(PolygonStamped, f'/{r}/local_costmap/published_footprint', lambda p, r=r: self.fp.__setitem__(r, p), 1),
                ]
                self.get_logger().info(f'subscribed {r}')

    def on_map(self, r, g):
        now = time.monotonic()
        if now - self.last.get(r, 0) < 1.0 or r not in self.odom:
            return
        self.last[r] = now
        o = self.odom[r].pose.pose
        px, py, res = o.position.x, o.position.y, g.info.resolution
        W, H = g.info.width, g.info.height
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        half = int(0.5 / res)
        cx, cy = int((px - ox) / res), int((py - oy) / res)
        rows = []
        for j in range(cy + half, cy - half - 1, -1):          # 위(+y)부터
            rows.append([int(g.data[j * W + i]) if 0 <= i < W and 0 <= j < H else -1 for i in range(cx - half, cx + half + 1)])
        self.buf[r].append({'t': time.time(), 'res': res, 'robot': [px, py, yaw_of(o.orientation)],
                            'origin_xy': [ox + (cx - half) * res, oy + (cy - half) * res],
                            'footprint': [[p.x, p.y] for p in self.fp[r].polygon.points] if r in self.fp else None,
                            'grid_top_to_bottom': rows})

    def on_task(self, m):
        if m.state == 'FAILED' and m.robot_id in self.buf:
            p = os.path.join(self.out, f'{m.mission_id}_{m.robot_id}.json')
            with open(p, 'w') as f:
                json.dump(list(self.buf[m.robot_id]), f)
            self.get_logger().info(f'saved {p} ({len(self.buf[m.robot_id])} frames)')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/pinky/fms_logs/costmap')
    rclpy.init()
    n = Rec(out)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
