#!/usr/bin/env python3
"""미션 결과 기록기 (v2). 주행 시험 동안 켜 두고, 끝난 뒤 summarize_missions.py / analyze_tolerance.py 로 분석한다.

기록하는 것 (JSONL, 한 줄이 한 이벤트):
  task    : /fleet/task_state 의 상태 전환 (ASSIGNED/RUNNING/SUCCEEDED/FAILED/CANCELED). RUNNING 피드백은 0.5초에 한 번만.
  goal    : /fleet/task 의 목표 좌표 (x, y, yaw) — 미션별 목표
  status  : /fleet/robot_status (주행 중 0.5초 간격, 상태 전환은 항상)
  amcl    : 로봇의 /<로봇>/amcl_pose (주행 중 0.5초 간격) — 위치, 방향, 공분산(불확실도)
  final   : 미션이 끝난 순간(SUCCEEDED/FAILED/CANCELED)의 최신 amcl 위치와 불확실도

사용 (관제PC, fms_env.sh 적용한 터미널):
  python3 <저장소>/tools/record_missions.py [출력파일]
Ctrl+C 로 종료. 기본 출력: ~/pinky/fms_logs/missions_<시작시각>.jsonl
"""
import json
import math
import os
import re
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from pinky_fms_interfaces.msg import RobotStatus, Task, TaskState

FINAL = ('SUCCEEDED', 'FAILED', 'CANCELED')
AMCL_RE = re.compile(r'^/([a-z][a-z0-9_]*)/amcl_pose$')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, path):
        super().__init__('mission_recorder')
        self.f = open(path, 'a', buffering=1, encoding='utf-8')
        self.busy = set()
        self.last = {}            # 샘플링용: 키 -> 마지막 기록 시각
        self.amcl = {}            # 로봇 -> 최신 amcl 레코드
        self.amcl_subs = {}
        self.create_subscription(TaskState, '/fleet/task_state', self.on_task, 100)
        self.create_subscription(Task, '/fleet/task', self.on_goal, 100)
        self.create_subscription(RobotStatus, '/fleet/robot_status', self.on_status, 100)
        self.create_timer(3.0, self.discover)
        self.get_logger().info(f'recording to {path}')

    def write(self, rec):
        rec['t'] = round(time.time(), 3)
        self.f.write(json.dumps(rec, ensure_ascii=False) + '\n')

    def due(self, key, period=0.5):
        now = time.monotonic()
        if now - self.last.get(key, 0) >= period:
            self.last[key] = now
            return True
        return False

    # ---- 로봇 amcl_pose 자동 구독 ----
    def discover(self):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name, _ in self.get_topic_names_and_types():
            m = AMCL_RE.match(name)
            if m and m.group(1) not in self.amcl_subs:
                rid = m.group(1)
                self.amcl_subs[rid] = self.create_subscription(
                    PoseWithCovarianceStamped, name, lambda msg, rid=rid: self.on_amcl(rid, msg), qos)
                self.get_logger().info(f'subscribed {name}')

    def on_amcl(self, rid, m):
        p, c = m.pose.pose.position, m.pose.covariance
        rec = {'kind': 'amcl', 'robot': rid, 'x': round(p.x, 4), 'y': round(p.y, 4), 'yaw': round(yaw_of(m.pose.pose.orientation), 4),
               'cov_xx': round(c[0], 5), 'cov_yy': round(c[7], 5), 'cov_yawyaw': round(c[35], 5)}
        self.amcl[rid] = dict(rec, t_msg=time.time())
        if rid in self.busy and self.due(('amcl', rid)):
            self.write(dict(rec))

    # ---- 미션 ----
    def on_goal(self, m):
        g = m.goal.pose
        self.write({'kind': 'goal', 'mission': m.mission_id, 'task': m.task_id, 'robot': m.robot_id, 'type': m.type,
                    'x': round(g.position.x, 4), 'y': round(g.position.y, 4), 'yaw': round(yaw_of(g.orientation), 4)})

    def on_task(self, m):
        if m.state == 'RUNNING' and not self.due(('task', m.mission_id)):
            return                               # 피드백은 0.5초에 한 번만
        self.write({'kind': 'task', 'mission': m.mission_id, 'task': m.task_id, 'robot': m.robot_id,
                    'state': m.state, 'dist': round(m.distance_remaining, 3), 'msg': m.message})
        if m.state in FINAL:
            a = self.amcl.get(m.robot_id)
            if a:
                self.write({'kind': 'final', 'mission': m.mission_id, 'robot': m.robot_id, 'state': m.state,
                            'amcl_age': round(time.time() - a['t_msg'], 2),
                            **{k: a[k] for k in ('x', 'y', 'yaw', 'cov_xx', 'cov_yy', 'cov_yawyaw')}})

    def on_status(self, m):
        if (m.state == 'BUSY' or m.robot_id in self.busy) and self.due(('status', m.robot_id)):
            self.write({'kind': 'status', 'robot': m.robot_id, 'state': m.state, 'x': round(m.x, 3), 'y': round(m.y, 3),
                        'yaw': round(m.yaw, 3), 'loc': m.localized, 'batt': round(m.battery_percent, 1)})
        (self.busy.add if m.state == 'BUSY' else self.busy.discard)(m.robot_id)


def main():
    default = os.path.expanduser(f"~/pinky/fms_logs/missions_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    path = sys.argv[1] if len(sys.argv) > 1 else default
    os.makedirs(os.path.dirname(path), exist_ok=True)
    rclpy.init()
    node = Recorder(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
