"""lane_traffic 시험용 가짜 차선 로봇 (카메라·라이다 없음, 질점 + 제자리 회전).

차선 지도(lanes_yaml)의 차선 띠 가운데를 따라 start → goal 로 한 번 간다 (편도). fms_lane_mission 의
lane_status / lane_traffic_cmd 인터페이스(2026-10-07 pinky-96 구현)를 흉내 낸다:
  - 앞 사물 정지: 다른 가짜 로봇(peers 의 lane_status pose)이 앞 ±0.10 m 통로 안 stop_dist(앞면 기준) 이내면 정지
  - hold / yield{x,y} / resume{role}, 같은 id 무시, HOLD·YIELDED 에서 10 s 무명령이면 resume 으로 간주
  - 비켜서기·복귀: 방향 오차 0.25 rad 초과면 제자리 회전, 아니면 직진 0.06 m/s, 0.03 m 안이면 도착, 복귀는 yaw ±8°
  - 갈림길(jn_wait, pinky-96 2026-10-08 사양 흉내): junctions 중심 0.20 m 안에 처음 들어오면 정지(detail 'junction check'),
    경로 출구 쪽 0.6 m(폭 ±0.20 m)에 다른 로봇이 없는 상태가 1 s 이어지면 출발, 있으면 'junction busy · robot'
'collision' 은 두 로봇 중심이 몸체 폭(0.12 m)보다 가까워진 횟수 (lane_status 에 기록, 시나리오가 확인).
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .gridmap import GridMap
from .planner import astar

DT = 0.05


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class LaneMockRobot(Node):
    def __init__(self):
        super().__init__('lane_mock_robot')
        for n, v in (('namespace', 'amr_01'), ('lanes_yaml', ''), ('start', [0.0, 0.0]), ('goal', [0.0, 0.0]),
                     ('peers', ['']), ('speed', 0.15), ('stop_dist', 0.15), ('start_delay', 3.0),
                     ('junctions', [0.0])):
            self.declare_parameter(n, v)
        gp = lambda n: self.get_parameter(n).value
        self.ns = gp('namespace')
        gm = GridMap(gp('lanes_yaml'), 0.02)
        s, g = list(gp('start')), list(gp('goal'))
        self.path = astar(gm, s, g, mask=gm.free)
        if self.path is None:
            raise RuntimeError('차선 위 경로 없음')
        self.i = 0
        d = self.path[min(3, len(self.path) - 1)] - self.path[0]
        self.x, self.y, self.yaw = float(self.path[0, 0]), float(self.path[0, 1]), math.atan2(d[1], d[0])
        self.speed, self.stop_dist = gp('speed'), gp('stop_dist')
        jv = list(gp('junctions'))
        self.junctions = [(jv[k], jv[k + 1]) for k in range(0, len(jv) - 1, 2)]
        self.jn_done = set()            # 이미 지나간 갈림길 (같은 갈림길에서 다시 멈추지 않게)
        self.jn = None                  # 지금 정지 중인 갈림길 index
        self.jn_clear_t = None
        self.t_start = time.monotonic() + gp('start_delay')
        self.state, self.detail = 'DRIVING', 'lane → goal'
        self.traffic = self.traffic_id = self.role = None
        self.role_t = 0.0
        self.rx = 0.0
        self.target = self.tyaw = self.rejoin = None
        self.blocked_t = None
        self.t0 = 0.0
        self.collisions = 0
        self.in_contact = False
        self.peers = {}
        for p in gp('peers'):
            if p and p != self.ns:
                self.create_subscription(String, f'/{p}/lane_status', lambda m, p=p: self._on_peer(p, m), 10)
        self.pub = self.create_publisher(String, f'/{self.ns}/lane_status', 10)
        self.create_subscription(String, f'/{self.ns}/lane_traffic_cmd', self._on_cmd, 10)
        self.create_timer(DT, self._step)
        self.create_timer(0.2, self._publish)
        self.get_logger().info(f'{self.ns}: {s} → {g} 경로 {len(self.path)} 점')

    def _on_peer(self, p, m):
        try:
            d = json.loads(m.data)
            if d.get('pose'):
                self.peers[p] = d['pose']
        except ValueError:
            pass

    def _on_cmd(self, m):
        d = json.loads(m.data)
        now = time.monotonic()
        self.rx = now
        if d['id'] == self.traffic_id:
            return
        self.traffic_id = d['id']
        if self.state == 'ARRIVED' and d['cmd'] == 'hold':
            return                      # 서 있는 로봇: hold 는 무시, yield/resume 은 따른다 (pinky-96 2026-10-08)
        if d['cmd'] == 'hold':
            if self.traffic in (None, 'HOLD'):
                self.traffic = 'HOLD'
        elif d['cmd'] == 'yield':
            if self.traffic not in ('YIELDING', 'YIELDED', 'REJOINING', 'YIELD_FAILED'):
                self.rejoin = (self.x, self.y, self.yaw)
            self.traffic, self.target, self.tyaw, self.t0, self.blocked_t = 'YIELDING', (d['x'], d['y']), None, now, None
        elif d['cmd'] == 'resume':
            self.role, self.role_t = d.get('role'), now
            if all(k in d for k in ('x', 'y', 'yaw')) and self.traffic in ('YIELDING', 'YIELDED', 'YIELD_FAILED', 'REJOINING'):
                self.rejoin = (d['x'], d['y'], d['yaw'])     # 복귀 자리 지정 (원래 자리가 막혔을 때)
                if self.traffic == 'REJOINING':
                    self.traffic = 'YIELD_FAILED'           # 아래 _resume 이 새 목표로 다시 시작
            self._resume(now)

    def _resume(self, now):
        if self.traffic == 'HOLD':
            self.traffic = None
        elif self.traffic in ('YIELDING', 'YIELDED', 'YIELD_FAILED') and self.rejoin is not None:
            self.traffic, self.target, self.tyaw, self.t0, self.blocked_t = 'REJOINING', self.rejoin[:2], self.rejoin[2], now, None

    def _front_block(self):
        """앞 ±0.10 m 통로 안 다른 로봇까지의 거리 (앞면 기준, 상대 몸체 반폭 0.06 빼고)"""
        best = float('inf')
        for p in self.peers.values():
            dx, dy = p[0] - self.x, p[1] - self.y
            fx = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
            fy = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
            if fx > 0 and abs(fy) <= 0.10 + 0.06:
                best = min(best, fx - 0.06 - 0.07)
        return best

    def _step(self):
        now = time.monotonic()
        for p in self.peers.values():           # 충돌 기록 (몸체 폭보다 가까움)
            close = math.hypot(p[0] - self.x, p[1] - self.y) < 0.12
            if close and not self.in_contact:
                self.collisions += 1
                self.get_logger().error(f'💥 {self.ns} 충돌 ({math.hypot(p[0] - self.x, p[1] - self.y):.3f} m)')
            self.in_contact = close
        if now < self.t_start:
            return
        if self.traffic is not None:
            return self._traffic_step(now)
        if self.state == 'ARRIVED':
            return
        if self._junction_wait(now):
            return
        if self._front_block() <= self.stop_dist:
            self.detail = 'obstacle ahead'
            return
        self.detail = 'lane → goal'
        if self.i >= len(self.path) - 1:
            self.state, self.detail = 'ARRIVED', 'at target'
            self.get_logger().info(f'🏁 {self.ns} 도착')
            return
        # 차선 추종 흉내: 경로 다음 점 쪽으로 방향을 맞추며 전진 (실제 차선 추종처럼 진행 방향은 경로를 따른다)
        step = self.speed * DT
        while step > 0 and self.i < len(self.path) - 1:
            nx, ny = self.path[self.i + 1]
            d = math.hypot(nx - self.x, ny - self.y)
            if d <= step:
                self.x, self.y, step = float(nx), float(ny), step - d
                self.i += 1
            else:
                self.x += (nx - self.x) / d * step
                self.y += (ny - self.y) / d * step
                step = 0
        j = min(self.i + 4, len(self.path) - 1)
        if j > self.i:
            self.yaw = math.atan2(self.path[j, 1] - self.y, self.path[j, 0] - self.x)

    def _junction_wait(self, now):
        if self.jn is None:
            for k, c in enumerate(self.junctions):
                if k not in self.jn_done and math.hypot(c[0] - self.x, c[1] - self.y) <= 0.20:
                    self.jn, self.jn_clear_t = k, None
                    break
            if self.jn is None:
                return False
        # 출구 쪽: 경로 앞 0.6 m 의 점들 주변 0.20 m 안에 다른 로봇이 있으면 busy
        seg = np.hypot(*np.diff(self.path[self.i:], axis=0).T)
        n = int(np.searchsorted(np.cumsum(seg), 0.6)) + 1
        ahead = self.path[self.i:self.i + n + 1]
        busy = any(np.min(np.hypot(ahead[:, 0] - p[0], ahead[:, 1] - p[1])) <= 0.20 for p in self.peers.values()) if len(ahead) else False
        if busy:
            self.jn_clear_t = None
            self.detail = 'junction busy · robot'
            return True
        self.jn_clear_t = self.jn_clear_t or now
        if now - self.jn_clear_t < 1.0:
            self.detail = 'junction check'
            return True
        self.jn_done.add(self.jn)
        self.jn = None
        return False

    def _traffic_step(self, now):
        tr = self.traffic
        if tr in ('HOLD', 'YIELDED') and now - self.rx > 10.0:
            self.role = None
            return self._resume(now)
        if tr not in ('YIELDING', 'REJOINING'):
            return
        if now - self.t0 > 20.0:
            self.traffic = 'YIELD_FAILED'
            return
        tx, ty = self.target
        d = math.hypot(tx - self.x, ty - self.y)
        if d <= 0.03:
            if self.tyaw is not None and abs(wrap(self.tyaw - self.yaw)) > math.radians(8):
                self.yaw += max(-0.8 * DT, min(0.8 * DT, wrap(self.tyaw - self.yaw)))
                return
            if tr == 'YIELDING':
                self.traffic, self.rx = 'YIELDED', now
            else:
                self.traffic, self.rejoin = None, None
                k = int(np.argmin(np.hypot(self.path[:, 0] - self.x, self.path[:, 1] - self.y)))
                self.i = max(self.i, k)              # 다른 자리로 복귀했으면 그 앞 경로부터 (차선 추종 흉내)
            return
        err = wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        if abs(err) > 0.25:
            self.yaw += max(-0.8 * DT, min(0.8 * DT, err))
            return
        if self._front_block() < 0.06:
            self.blocked_t = self.blocked_t or now
            if now - self.blocked_t > 3.0:
                self.traffic = 'YIELD_FAILED'
            return
        self.blocked_t = None
        self.yaw += err * 0.5
        v = min(0.06, d) * DT
        self.x += v * math.cos(self.yaw)
        self.y += v * math.sin(self.yaw)

    def _publish(self):
        if self.state == 'ARRIVED' and self.traffic is None:
            state, detail = 'ARRIVED', 'at target'
        elif self.traffic is not None:
            state, detail = {'HOLD': ('WAITING', 'encounter'), 'YIELDING': ('WAITING', 'yielding'),
                             'YIELDED': ('WAITING', 'yielded'), 'REJOINING': ('RESUME DRIVING', 'back to lane'),
                             'YIELD_FAILED': ('WAITING', 'yield failed')}[self.traffic]
        elif self.detail == 'obstacle ahead' or self.detail.startswith('junction'):
            state, detail = 'WAITING', self.detail
        elif self.role == 'priority' and time.monotonic() - self.role_t < 5.0:
            state, detail = 'OWN DRIVING PRIORITY', 'lane → goal'
        else:
            state, detail = 'DRIVING', 'lane → goal'
        rem = float(np.hypot(*np.diff(self.path[self.i:], axis=0).T).sum()) if self.i < len(self.path) - 1 else 0.0
        self.pub.publish(String(data=json.dumps({
            'robot': self.ns, 'mission': 'lane', 'state': state, 'detail': detail, 'dist': round(rem, 2),
            'pose': [round(self.x, 3), round(self.y, 3), round(self.yaw, 3)], 'localized': True,
            'traffic': self.traffic, 'traffic_id': self.traffic_id, 'role': self.role, 'collisions': self.collisions})))


def main():
    rclpy.init()
    node = LaneMockRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):     # Ctrl+C / launch 종료
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
