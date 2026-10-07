#!/usr/bin/env python3
"""위치 추정 launch 확인 (run_localization_test.sh 가 부른다. 설치하지 않는다).

항목마다 PASS/FAIL 을 출력하고, 실패가 하나라도 있으면 종료 코드 1.
  1. map_server, amcl 이 lifecycle active
  2. 파라미터가 실제로 적용됨 (덮어쓴 max_particles, scan_topic, 프레임 이름)
  3. 초기 위치(참 위치에서 10 cm·8° 어긋나게) → request_nomotion_update 40번 → amcl_pose 가 참 위치 5 cm 안으로 수렴
  4. /<ns>/tf 에 map→odom, /<ns>/map 발행, 전역 /map 없음
  5. /<ns>/cmd_vel 발행자 0개 + Nav2 주행 노드 없음 (차선 모드에서 cmd_vel 이 겹치지 않는다는 설계)
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage

NAV_NODES = ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'velocity_smoother',
             'waypoint_follower', 'smoother_server')
LOC_NODES = ('map_server', 'amcl', 'lifecycle_manager_localization')
# 초기 위치를 참 위치에서 10 cm·8° 어긋나게 주고, 제자리 갱신 뒤 위치가 5 cm 안으로 들어오는지 본다
# (스캔을 쓰지 않으면 10 cm 그대로라 분명히 FAIL). 방향은 로봇이 움직이지 않으면 잘 맞춰지지 않으므로
# (실험: 8° → 5~9°) 프레임·부호 실수 같은 큰 오류만 잡도록 12° 로 둔다.
ERR_XY, ERR_YAW = 0.05, math.radians(12.0)
INIT_OFFSET = (0.08, -0.06, math.radians(8.0))
NOMOTION_UPDATES = 40


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self, ns):
        super().__init__('localization_check')
        self.ns = '/' + ns.strip('/')
        self.ok = []
        self.pose = None
        self.tf_pairs = set()
        self.create_subscription(PoseWithCovarianceStamped, f'{self.ns}/amcl_pose', self._on_pose, 10)
        self.create_subscription(TFMessage, f'{self.ns}/tf', self._on_tf, 100)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, f'{self.ns}/initialpose', 10)

    def _on_pose(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_tf(self, m):
        for t in m.transforms:
            self.tf_pairs.add((t.header.frame_id, t.child_frame_id))

    def record(self, ok, name, detail=''):
        print(f'[{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''), flush=True)
        self.ok.append(bool(ok))

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, srv_type, name, req, timeout=5.0):
        cli = self.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result()
        finally:
            self.destroy_client(cli)

    def wait_active(self, node, timeout):
        end = time.monotonic() + timeout
        label = ''
        while time.monotonic() < end:
            res = self.call(GetState, f'{self.ns}/{node}/get_state', GetState.Request(), timeout=2.0)
            label = res.current_state.label if res else '(응답 없음)'
            if label == 'active':
                return True, label
            self.spin_for(0.5)
        return False, label

    def params(self, node, names):
        res = self.call(GetParameters, f'{self.ns}/{node}/get_parameters', GetParameters.Request(names=list(names)))
        if res is None:
            return {}
        out = {}
        for n, v in zip(names, res.values):
            out[n] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value}.get(v.type)
        return out

    def send_initial_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.01                     # 표준편차 0.1 m
        cov[35] = math.radians(10.0) ** 2          # 표준편차 10°
        m.pose.covariance = cov
        self.init_pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--namespace', default='amr_01')
    ap.add_argument('--pose', required=True, help='참 위치 x,y,yaw')
    ap.add_argument('--frame-prefix', default='')
    ap.add_argument('--scan', default='scan', help='AMCL 이 써야 하는 스캔 토픽')
    ap.add_argument('--max-particles', type=int, default=500)
    ap.add_argument('--expect-filter', action='store_true', help='robot_scan_filter 가 떠 있어야 함')
    args = ap.parse_args()
    tx, ty, tyaw = (float(v) for v in args.pose.split(','))
    pre = args.frame_prefix

    rclpy.init()
    c = Check(args.namespace)
    ns = c.ns

    # 1. lifecycle
    for node in ('map_server', 'amcl'):
        ok, label = c.wait_active(node, timeout=40.0)
        c.record(ok, f'{ns}/{node} lifecycle active', label)

    # 2. 파라미터 (전체 이름 키 + 덮어쓰기 + 접두어가 실제로 적용됐는가)
    want = {'max_particles': args.max_particles, 'scan_topic': args.scan, 'base_frame_id': pre + 'base_footprint',
            'odom_frame_id': pre + 'odom', 'global_frame_id': 'map'}
    got = c.params('amcl', list(want))
    for k, v in want.items():
        c.record(got.get(k) == v, f'amcl 파라미터 {k}', f'기대 {v!r}, 실제 {got.get(k)!r}')

    # 3. 초기 위치 → 제자리 갱신 반복 → 수렴
    end = time.monotonic() + 10.0
    while c.count_subscribers(f'{ns}/initialpose') < 1 and time.monotonic() < end:
        c.spin_for(0.2)
    ix, iy, iyaw = tx + INIT_OFFSET[0], ty + INIT_OFFSET[1], tyaw + INIT_OFFSET[2]
    for _ in range(2):
        c.send_initial_pose(ix, iy, iyaw)
        c.spin_for(0.5)
    n_ok = 0
    for _ in range(NOMOTION_UPDATES):
        n_ok += c.call(Empty, f'{ns}/request_nomotion_update', Empty.Request(), timeout=2.0) is not None
        c.spin_for(0.2)
    c.spin_for(1.0)
    c.record(n_ok == NOMOTION_UPDATES, f'{ns}/request_nomotion_update 서비스', f'{n_ok}/{NOMOTION_UPDATES} 번 응답')
    if c.pose is None:
        c.record(False, f'{ns}/amcl_pose 수신', '받지 못함')
    else:
        exy = math.hypot(c.pose[0] - tx, c.pose[1] - ty)
        eyaw = abs(wrap(c.pose[2] - tyaw))
        c.record(exy <= ERR_XY and eyaw <= ERR_YAW, 'amcl_pose 가 참 위치로 수렴',
                 f'위치 오차 {exy * 100:.1f} cm (허용 {ERR_XY * 100:.0f}), 방향 오차 {math.degrees(eyaw):.1f}° '
                 f'(허용 {math.degrees(ERR_YAW):.0f}) — 초기 오차 {math.hypot(*INIT_OFFSET[:2]) * 100:.1f} cm, '
                 f'{math.degrees(INIT_OFFSET[2]):.0f}°')

    # 4. TF / 지도 토픽
    c.record(('map', pre + 'odom') in c.tf_pairs, f'{ns}/tf 에 map → {pre}odom', f'본 변환: {sorted(c.tf_pairs)}')
    c.record(c.count_publishers(f'{ns}/map') >= 1, f'{ns}/map 발행')
    c.record(c.count_publishers('/map') == 0, '전역 /map 발행자 없음 (namespace 밖으로 새지 않음)')

    # 5. cmd_vel 이 겹치지 않는다는 설계: 위치 추정 launch 는 cmd_vel 을 내지 않는다
    n_cmd = c.count_publishers(f'{ns}/cmd_vel')
    c.record(n_cmd == 0, f'{ns}/cmd_vel 발행자 0개', f'{n_cmd}개')
    nodes = {n for n, s in c.get_node_names_and_namespaces() if s == ns}
    c.record(not nodes & set(NAV_NODES), 'Nav2 주행 노드 없음', f'{ns} 의 노드: {sorted(nodes)}')
    c.record(set(LOC_NODES) <= nodes, '위치 추정 노드 3개 실행 중', ', '.join(LOC_NODES))
    has_filter = 'robot_scan_filter' in nodes
    c.record(has_filter == args.expect_filter, f'robot_scan_filter {"있음" if args.expect_filter else "없음"}',
             f'실제 {"있음" if has_filter else "없음"}')
    if args.expect_filter:
        c.record(c.count_publishers(f'{ns}/scan_robotfree') >= 1 and c.count_subscribers(f'{ns}/scan_robotfree') >= 1,
                 f'{ns}/scan_robotfree: 필터가 내고 AMCL 이 받음')

    passed = sum(c.ok)
    print(f'== {passed}/{len(c.ok)} PASS', flush=True)
    c.destroy_node()
    rclpy.try_shutdown()
    return 0 if all(c.ok) else 1


if __name__ == '__main__':
    sys.exit(main())
