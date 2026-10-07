"""차선 미션 경로 따라가기 계산 (로봇, ROS 없음). fms_lane_mission 이 쓴다.

관제 lane_route(pinky_fms_lane)가 lane_cmd 에 붙인 route / route_back 필드를 읽어
  - 지금 위치의 경로 진행 거리 s 와 경로에서 벗어난 거리
  - 다음 갈림길 동작 (갈림길 구역에 들어오면 시작)
  - 도착 (s 가 s_goal - arrive_tol 이상)
을 계산한다. 필드 형식: {"points": [[x, y], ...], "s_goal": m, "uturn": bool,
  "maneuvers": [{"node", "x", "y", "s_at", "radius", "turn": STRAIGHT|LEFT|RIGHT, "follow": LEFT|RIGHT, "follow_dist"}]}
"""
import math

MAX_POINTS = 2000
MAX_MANEUVERS = 50
TURNS = ('STRAIGHT', 'LEFT', 'RIGHT')
FOLLOWS = ('LEFT', 'RIGHT')


class RouteError(ValueError):
    """route 필드가 잘못됨."""


def _num(v, name):
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise RouteError(f'{name} 가 숫자가 아님')
    if not math.isfinite(f) or abs(f) > 1000.0:
        raise RouteError(f'{name} 값이 범위를 벗어남')
    return f


class RouteTrack:
    def __init__(self, d, arrive_tol=0.05, window_back=0.3, window_ahead=0.6):
        if not isinstance(d, dict):
            raise RouteError('route 가 객체가 아님')
        pts = d.get('points')
        if not isinstance(pts, list) or not 2 <= len(pts) <= MAX_POINTS:
            raise RouteError(f'points 는 2~{MAX_POINTS}개여야 함')
        self.pts = []
        for i, p in enumerate(pts):
            if not isinstance(p, (list, tuple)) or len(p) != 2:
                raise RouteError(f'points[{i}] 는 [x, y] 여야 함')
            self.pts.append((_num(p[0], f'points[{i}].x'), _num(p[1], f'points[{i}].y')))
        self.cum = [0.0]
        for a, b in zip(self.pts, self.pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.cum[-1]
        self.s_goal = min(max(_num(d.get('s_goal', self.length), 's_goal'), 0.0), self.length)
        self.uturn = bool(d.get('uturn', False))
        mans = d.get('maneuvers') or []
        if not isinstance(mans, list) or len(mans) > MAX_MANEUVERS:
            raise RouteError('maneuvers 가 목록이 아니거나 너무 많음')
        self.maneuvers = []
        for i, m in enumerate(mans):
            if not isinstance(m, dict) or m.get('turn') not in TURNS or m.get('follow') not in FOLLOWS:
                raise RouteError(f'maneuvers[{i}]: turn 은 {TURNS}, follow 는 {FOLLOWS}')
            self.maneuvers.append({'node': str(m.get('node', ''))[:32], 'turn': m['turn'], 'follow': m['follow'],
                                   's_at': _num(m.get('s_at'), f'maneuvers[{i}].s_at'),
                                   'radius': max(0.0, _num(m.get('radius', 0.2), f'maneuvers[{i}].radius')),
                                   'follow_dist': max(0.0, _num(m.get('follow_dist', 0.3), f'maneuvers[{i}].follow_dist'))})
        self.maneuvers.sort(key=lambda m: m['s_at'])
        self.arrive_tol = max(0.0, float(arrive_tol))
        self.back, self.ahead = window_back, window_ahead
        self.s, self.lat = 0.0, 0.0
        self.next_i = 0

    def _project(self, x, y, lo, hi):
        best = None
        for i, (a, b) in enumerate(zip(self.pts, self.pts[1:])):
            if self.cum[i + 1] < lo or self.cum[i] > hi:
                continue
            dx, dy = b[0] - a[0], b[1] - a[1]
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else min(max(((x - a[0]) * dx + (y - a[1]) * dy) / L2, 0.0), 1.0)
            s = self.cum[i] + t * (self.cum[i + 1] - self.cum[i])
            if not lo <= s <= hi:
                s = min(max(s, lo), hi)
                t = 0.0 if self.cum[i + 1] == self.cum[i] else (s - self.cum[i]) / (self.cum[i + 1] - self.cum[i])
            qx, qy = a[0] + t * dx, a[1] + t * dy
            d = math.hypot(x - qx, y - qy)
            if best is None or d < best[0]:
                best = (d, s)
        return best

    def update(self, x, y):
        """지금 위치 → (s, 경로까지 거리). 이전 s 근처(뒤 window_back, 앞 window_ahead)에서 먼저 찾아 나란한 다른 구간으로
        튀지 않게 하고, 그 근처에서 멀면(양보 뒤 다른 자리로 복귀 등) 경로 전체에서 다시 찾는다."""
        near = self._project(x, y, self.s - self.back, self.s + self.ahead)
        if near is None or near[0] > 0.3:
            far = self._project(x, y, 0.0, self.length)
            if far is not None and (near is None or far[0] < near[0] - 0.1):
                near = far
        self.lat, self.s = near[0], near[1]
        return self.s, self.lat

    def due_maneuver(self):
        """갈림길 구역에 들어왔으면 그 동작 (이미 구역을 지나쳤으면 건너뛴다). 없으면 None."""
        while self.next_i < len(self.maneuvers):
            m = self.maneuvers[self.next_i]
            if self.s > m['s_at'] + m['radius']:
                self.next_i += 1                 # 양보·복귀 등으로 이미 지나침: 늦게 꺾지 않는다
                continue
            return m if self.s >= m['s_at'] - m['radius'] else None
        return None

    def done_maneuver(self):
        self.next_i += 1

    def next_node(self):
        return self.maneuvers[self.next_i]['node'] if self.next_i < len(self.maneuvers) else None

    def arrived(self):
        return self.s >= self.s_goal - self.arrive_tol
