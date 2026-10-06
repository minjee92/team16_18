"""ROS 통합 시험 드라이버 (도메인 91 격리 환경에서 run_ros_test.sh 가 실행).
  python3 tests/ros_scenario.py <시나리오> [--gz] : headon | follow | parked ...
  --gz : 가제보 로봇일 때. 시작 위치로 로봇을 순간이동(gz set_pose)시키고, 초기 위치는 목표 쪽을 보는 방향으로 준다.
미션을 보내고, 두 로봇의 amcl_pose 로 최소 간격·충돌(중심 거리 < 0.19 m)을 센다.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node

from pinky_fms_interfaces.msg import MissionRequest, TaskState

SC = {
    # (로봇, 시작, 목표)
    'headon': [('amr_01', (1.97, 0.13), (0.20, 0.15)), ('amr_02', (0.20, 0.40), (1.95, 0.35))],
    'follow': [('amr_01', (1.97, 0.13), (0.20, 0.15)), ('amr_02', (1.97, 0.45), (0.60, 0.28))],
    'parked': [('amr_01', (0.98, 1.0), None), ('amr_02', (1.97, 0.13), (0.20, 0.15))],
    'deadlock': [('amr_01', (1.85, 0.62), (2.04, 0.10)), ('amr_02', (1.90, 0.36), (0.82, 0.15))],     # 실물 교착 재현 (박스 옆 통로 정면)
    'close': [('amr_01', (1.88, 0.58), (2.04, 0.10)), ('amr_02', (1.90, 0.40), (0.82, 0.15))],     # 박스 옆에서 거의 붙은 채 마주침 (18:49 재현)
    'three': [('amr_01', (1.97, 0.13), (0.20, 0.15)), ('amr_02', (0.20, 0.40), (1.95, 0.40)), ('amr_03', (1.40, 0.95), None)],
    'wallgoal': [('amr_01', (1.20, 0.30), (0.96, 0.50)), ('amr_02', (0.30, 0.15), None)],     # 목표가 벽 속: SUCCEEDED 가 아니라 FAILED 여야 한다
    'nearwall': [('amr_01', (1.20, 0.30), (0.88, 0.50)), ('amr_02', (0.30, 0.15), None)],     # 벽 서쪽 8 cm: 갈 수는 있다 (벽을 돌아서)
}


class Driver(Node):
    def __init__(self, name):
        super().__init__('traffic_test_driver')
        self.spec = SC[name]
        self.pose = {}
        self.state = {}
        self.msg = {}
        self.min_sep, self.coll = 9.0, 0
        self.pair_min = {}
        self.coll_log = []
        self.pub_req = self.create_publisher(MissionRequest, '/fleet/mission_request', 10)
        self.pub_init = {rid: self.create_publisher(PoseWithCovarianceStamped, f'/{rid}/initialpose', 10) for rid, _, _ in self.spec}
        for rid, _, _ in self.spec:
            self.create_subscription(PoseWithCovarianceStamped, f'/{rid}/amcl_pose', lambda m, rid=rid: self.on_pose(rid, m), 10)
        self.create_subscription(TaskState, '/fleet/mission_state', self.on_state, 100)
        self.log = []

    def on_pose(self, rid, m):
        self.pose[rid] = (m.pose.pose.position.x, m.pose.pose.position.y)
        ps = list(self.pose.values())
        for i in range(len(ps)):
            for j in range(i + 1, len(ps)):
                d = math.hypot(ps[i][0] - ps[j][0], ps[i][1] - ps[j][1])
                self.min_sep = min(self.min_sep, d)
                k = tuple(sorted((list(self.pose)[i], list(self.pose)[j])))
                self.pair_min[k] = min(self.pair_min.get(k, 9.0), d)
                if d < 0.19:
                    self.coll += 1
                    if len(self.coll_log) < 3:
                        self.coll_log.append((time.monotonic(), k, round(d, 3), {x: tuple(round(v, 2) for v in self.pose[x]) for x in self.pose}))

    def on_state(self, m):
        k = (m.robot_id, m.state, m.message)
        if self.state.get(m.robot_id) != (m.state, m.message):
            self.log.append((time.monotonic(), m.robot_id, m.state, m.message))
        self.state[m.robot_id] = (m.state, m.message)

    @staticmethod
    def start_yaw(s, g):
        return math.atan2(g[1] - s[1], g[0] - s[0]) if g else 0.0

    def init_poses(self):
        for rid, s, g in self.spec:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x, msg.pose.pose.position.y = s
            yaw = self.start_yaw(s, g)
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02
            msg.pose.covariance[35] = 0.03
            self.pub_init[rid].publish(msg)

    def gz_teleport(self, world='fms_course'):
        """가제보 로봇을 시나리오 시작 위치로 옮긴다 (실물에서는 사람이 옮기는 것에 해당)"""
        for rid, s, g in self.spec:
            yaw = self.start_yaw(s, g)
            req = f'name: "{rid}" position: {{x: {s[0]} y: {s[1]} z: 0.03}} orientation: {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}'
            subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '3000', '--req', req], capture_output=True)

    def clear_costmaps(self):
        for rid, _, _ in self.spec:
            for cm in ('global_costmap/clear_entirely_global_costmap', 'local_costmap/clear_entirely_local_costmap'):
                subprocess.run(['ros2', 'service', 'call', f'/{rid}/{cm}', 'nav2_msgs/srv/ClearEntireCostmap', '{}'], capture_output=True, timeout=15)

    def send(self, rid, goal, mid):
        r = MissionRequest(mission_id=mid, type='NAV_GOTO', robot_id=rid)
        r.goal = PoseStamped()
        r.goal.header.frame_id = 'map'
        r.goal.pose.position.x, r.goal.pose.position.y = goal
        r.goal.pose.orientation.w = 1.0
        self.pub_req.publish(r)


def main():
    name = sys.argv[1]
    gz = '--gz' in sys.argv
    rclpy.init()
    d = Driver(name)
    if gz:
        d.gz_teleport()
        time.sleep(2.0)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 6:
        d.init_poses()
        rclpy.spin_once(d, timeout_sec=0.5)
    if gz:
        time.sleep(2.0)
        d.clear_costmaps()
        # Nav2 가 완전히 활성화될 때까지 (초기 위치 전에는 costmap 이 TF 를 기다려 bt_navigator 가 inactive)
        t1 = time.monotonic()
        while time.monotonic() - t1 < 90:          # fleet_traffic 이 RESET→STARTUP 으로 살려 내는 시간 포함
            st = [subprocess.run(['ros2', 'lifecycle', 'get', f'/{rid}/bt_navigator'], capture_output=True, text=True, timeout=10).stdout.strip() for rid, _, _ in d.spec]
            if all(x.startswith('active') for x in st):
                break
            time.sleep(2.0)
        print('bt_navigator:', st, f'({time.monotonic() - t1:.0f}s 대기)')
        time.sleep(1.0)
    goals = [(rid, g) for rid, _, g in d.spec if g]
    for rid, g in goals:
        d.send(rid, g, f'T_{name}_{rid}')
    t0 = time.monotonic()
    done = set()
    while time.monotonic() - t0 < 120 and len(done) < len(goals):
        rclpy.spin_once(d, timeout_sec=0.2)
        for rid, _ in goals:
            if d.state.get(rid, ('',))[0] in ('SUCCEEDED', 'FAILED', 'CANCELED'):
                done.add(rid)
    el = time.monotonic() - t0
    print(f'\n[{name}] 소요 {el:.1f}s  결과 {({k: v[0] for k, v in d.state.items()})}  최소 간격 {d.min_sep:.3f} m  충돌 프레임 {d.coll}')
    print('  쌍별 최소 간격', {f'{a}-{b}': round(v, 3) for (a, b), v in d.pair_min.items()})
    for c in d.coll_log:
        print(f'  충돌 +{c[0] - t0:.1f}s {c[1]} {c[2]} m 위치 {c[3]}')
    seen = None
    for t, rid, st, msg in d.log:
        if (rid, st, msg) != seen:
            print(f'  +{t - t0:5.1f}s {rid}: {st} {msg}')
            seen = (rid, st, msg)
    rclpy.shutdown()


main()
