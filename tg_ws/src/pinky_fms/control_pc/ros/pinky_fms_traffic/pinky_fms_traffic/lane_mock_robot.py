"""lane_traffic 시험용 가짜 차선 로봇 (카메라·라이다 없음, 질점 + 제자리 회전).

차선 지도(lanes_yaml)의 차선 띠 가운데를 따라 start → goal 로 한 번 간다 (편도). fms_lane_mission 의
lane_status / lane_traffic_cmd 인터페이스(2026-10-07 pinky-96 구현)를 흉내 낸다:
  - 앞 사물 정지: 다른 가짜 로봇(peers 의 lane_status pose)이 앞 ±0.10 m 통로 안 stop_dist(앞면 기준) 이내면 정지
  - hold / yield{x,y} / resume{role}, 같은 id 무시. HOLD·YIELDED 는 resume 으로만 풀림
    (traffic_cmd_timeout > 0 이면 그 시간 무명령일 때 resume 으로 간주. 로봇 fms_lane_mission 과 같은 기본 0 = 끔)
  - 비켜서기·복귀: 방향 오차 0.25 rad 초과면 제자리 회전, 아니면 직진 0.06 m/s, 0.03 m 안이면 도착, 복귀는 yaw ±8°
'collision' 은 두 로봇 중심이 몸체 폭(0.12 m)보다 가까워진 횟수 (lane_status 에 기록, 시나리오가 확인).
경로 모드 (route_mode:=true, 기본 false): start 에서 start_yaw 를 보고 IDLE 로 기다리다가 /<ns>/lane_cmd 를 받으면 간다.
  lane_cmd 에 route(관제 lane_route 가 붙인 경로)가 있으면 그 점들을 따라가고(출발 U턴 포함), go 의 route_back 이 있으면
  목적지에서 돌아온다. route 가 없으면 차선 지도 A* 로 목표까지 간다. cancel 이면 멈추고 IDLE. lane_status 에 cmd_id 를 낸다.
"""
import json
import math
import time

import numpy as np
import rclpy
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
                     ('traffic_cmd_timeout', 0.0), ('route_mode', False), ('start_yaw', 0.0)):
            self.declare_parameter(n, v)
        gp = lambda n: self.get_parameter(n).value
        self.ns = gp('namespace')
        self.gm = gm = GridMap(gp('lanes_yaml'), 0.02)
        s, g = list(gp('start')), list(gp('goal'))
        self.route_mode = bool(gp('route_mode'))
        self.cmd_id, self.route_back, self.leg = None, None, 'outbound'
        if self.route_mode:                              # lane_cmd 를 기다린다
            self.path, self.i = None, 0
            self.x, self.y, self.yaw = float(s[0]), float(s[1]), float(gp('start_yaw'))
            self.create_subscription(String, f'/{gp("namespace")}/lane_cmd', self._on_lane_cmd, 10)
        else:
            self.path = astar(gm, s, g, mask=gm.free)
            if self.path is None:
                raise RuntimeError('차선 위 경로 없음')
            self.i = 0
            d = self.path[min(3, len(self.path) - 1)] - self.path[0]
            self.x, self.y, self.yaw = float(self.path[0, 0]), float(self.path[0, 1]), math.atan2(d[1], d[0])
        self.speed, self.stop_dist = gp('speed'), gp('stop_dist')
        self.traffic_cmd_timeout = float(gp('traffic_cmd_timeout'))
        self.t_start = time.monotonic() + gp('start_delay')
        self.state, self.detail = ('IDLE', 'set goal') if self.route_mode else ('DRIVING', 'lane → goal')
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
        if self.path is not None:
            self.get_logger().info(f'{self.ns}: {s} → {g} 경로 {len(self.path)} 점')
        else:
            self.get_logger().info(f'{self.ns}: 경로 모드, ({self.x:.2f}, {self.y:.2f}) 에서 lane_cmd 대기')

    # ---------- 경로 모드 ----------
    @staticmethod
    def _dense(points, step=0.02):
        pts = np.asarray(points, float)
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(1, int(math.ceil(np.hypot(*(b - a)) / step)))
            out += [a + (b - a) * k / n for k in range(1, n + 1)]
        return np.array(out)

    def _start_leg(self, route):
        self.path, self.i = self._dense(route['points']), 0
        if route.get('uturn'):
            self.yaw = wrap(self.yaw + math.pi)          # 제자리 U턴 (가짜 로봇은 바로 돈다)
            self.get_logger().info(f'{self.ns}: 출발 U턴')

    def _on_lane_cmd(self, m):
        try:
            d = json.loads(m.data)
            cid, cmd = int(d['id']), d['cmd']
        except (ValueError, KeyError, TypeError):
            return
        if cid == self.cmd_id:
            return
        self.cmd_id = cid
        self.traffic = self.target = self.rejoin = None
        if cmd == 'cancel':
            self.path, self.state, self.detail = None, 'IDLE', 'canceled · set goal'
            return
        if cmd not in ('go', 'home'):
            return
        self.leg, self.route_back = 'outbound', (d.get('route_back') if cmd == 'go' else None)
        if 'route' in d:
            self._start_leg(d['route'])
        else:
            self.path = astar(self.gm, [self.x, self.y], [d['x'], d['y']], mask=self.gm.free)
            self.i = 0
            if self.path is None:
                self.state, self.detail = 'FAILED', 'no path'
                return
        self.state, self.detail = 'DRIVING', 'lane → goal'
        self.get_logger().info(f'{self.ns}: lane_cmd {cid} {cmd} → ({d["x"]:.2f}, {d["y"]:.2f}), 경로 {len(self.path)} 점'
                               + (', 돌아오는 길 있음' if self.route_back else ''))

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
        if self.state in ('ARRIVED',):
            return
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
        if self.state in ('ARRIVED', 'IDLE', 'FAILED') or now < self.t_start or self.path is None:
            return
        if self.traffic is not None:
            return self._traffic_step(now)
        if self._front_block() <= self.stop_dist:
            self.detail = 'obstacle ahead'
            return
        self.detail = 'lane → goal'
        if self.i >= len(self.path) - 1:
            if self.route_back is not None and self.leg == 'outbound':
                self.leg = 'inbound'
                self._start_leg(self.route_back)
                self.route_back = None
                self.get_logger().info(f'🎯 {self.ns} 목적지 도착 → 돌아오는 길')
                return
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

    def _traffic_step(self, now):
        tr = self.traffic
        if self.traffic_cmd_timeout > 0 and tr in ('HOLD', 'YIELDED') and now - self.rx > self.traffic_cmd_timeout:
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
        if self.state in ('ARRIVED', 'IDLE', 'FAILED'):
            state, detail = self.state, self.detail
        elif self.traffic is not None:
            state, detail = {'HOLD': ('WAITING', 'encounter'), 'YIELDING': ('WAITING', 'yielding'),
                             'YIELDED': ('WAITING', 'yielded'), 'REJOINING': ('RESUME DRIVING', 'back to lane'),
                             'YIELD_FAILED': ('WAITING', 'yield failed')}[self.traffic]
        elif self.detail == 'obstacle ahead':
            state, detail = 'WAITING', 'obstacle ahead'
        elif self.role == 'priority' and time.monotonic() - self.role_t < 5.0:
            state, detail = 'OWN DRIVING PRIORITY', 'lane → goal'
        else:
            state, detail = 'DRIVING', 'lane → goal'
        rem = (float(np.hypot(*np.diff(self.path[self.i:], axis=0).T).sum())
               if self.path is not None and self.i < len(self.path) - 1 else 0.0)
        self.pub.publish(String(data=json.dumps({
            'robot': self.ns, 'mission': 'lane', 'state': state, 'detail': detail, 'dist': round(rem, 2),
            'pose': [round(self.x, 3), round(self.y, 3), round(self.yaw, 3)], 'localized': True,
            'traffic': self.traffic, 'traffic_id': self.traffic_id, 'role': self.role, 'collisions': self.collisions,
            'cmd_id': self.cmd_id, 'leg': self.leg})))


def main():
    rclpy.init()
    node = LaneMockRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
