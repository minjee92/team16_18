"""차선 코스 모델 (순수 Python, ROS 를 쓰지 않는다).

코스 설정 파일(YAML, map 좌표)을 읽어 길(간선)·갈림길·구역을 만들고 다음을 한다.
  - locate()    : 로봇 위치·방향 → 어느 길의 어디쯤인지 (Location)
  - snap_goal() : 클릭한 목표 → 길 위의 점 (멀면 거부, 금지 구역 안이면 구역 밖으로 옮김)
  - plan()      : 시작 → 목표 경로 (꺾은선, 갈림길 동작, 출발 U턴 여부) (Route)
코스 모양(좌표)은 코드에 없고 모두 설정 파일에 있다. 실측값은 따로 기록한 파일(measured)이 추정값을 덮어쓴다.

용어
  간선(edge) : 길 가운데 선 꺾은선. points 순서가 + 방향. oneway 면 + 방향으로만 다닌다.
  s          : 간선(또는 경로) 첫 점부터 선을 따라 잰 거리 (m)
  dir        : +1 = points 순서 방향, -1 = 반대 방향, 0 = 모름 (로봇이 길과 거의 수직)
  갈림길     : 간선 끝이 여러 개 모이는 점. moves 표에 (들어온 간선 → 나갈 간선) 별 동작이 있다
  막다른 끝  : 간선 끝 중 갈림길이 아닌 곳 (꼬리 끝). 경로는 여기서 더 갈 수 없다
"""
import heapq
import itertools
import math
import os
from dataclasses import dataclass

import numpy as np
import yaml

TURNS = ('STRAIGHT', 'LEFT', 'RIGHT')       # pinky_fms_lane_interfaces/LaneManeuver 상수와 같은 값
FOLLOWS = ('LEFT', 'RIGHT')
SOURCES = ('estimate', 'measured')
ALIGN_MIN = 0.5      # 로봇 방향과 길 방향의 cos 이 이보다 작으면(60° 넘게 어긋남) 방향을 모른다고 본다
WAIT_ON_ROAD = 0.05  # 대기 지점은 길 가운데 선에서 이 거리(m) 안에 있어야 한다
EPS = 1e-6

DEFAULT_PARAMS = {
    'snap_max': 0.15,            # 클릭한 목표를 길 위로 맞출 최대 거리 (m). 더 멀면 거부
    'dead_end_clearance': 0.15,  # 막다른 끝에서 이 거리 안에는 목표를 두지 않는다 (m)
    'wait_keepout': 0.15,        # 대기 지점 앞뒤 이 거리 안에는 목표를 두지 않는다 (m)
    'uturn_clearance': 0.10,     # 제자리 U턴에 필요한 벽까지 최소 거리 (m)
    'uturn_penalty': 0.5,        # 경로를 고를 때 U턴 1번을 이만큼(m) 먼 것으로 친다
    'lane_width': 0.14,          # 길 폭 (m). 그림·횡단보도 판정용
}


class CourseError(ValueError):
    """코스 설정 오류. problems 에 사람이 읽을 이유 목록이 있다."""

    def __init__(self, problems):
        super().__init__('코스 설정 오류:\n  - ' + '\n  - '.join(problems))
        self.problems = list(problems)


class PlanError(ValueError):
    """경로를 만들 수 없음. 메시지는 GUI 에 그대로 보여 줄 수 있는 문장이다."""


@dataclass
class Point:
    name: str
    x: float
    y: float
    yaw: float = None            # 출발점처럼 방향이 있는 점만 (rad)
    source: str = 'estimate'     # estimate | measured
    stamp: str = ''              # 기록 시각 (measured)


@dataclass
class Location:
    """길 위의 한 위치."""
    edge: str
    s: float                     # 간선 기준 거리 (m)
    dir: int                     # 로봇이 보는 방향 (+1/-1, 0 = 모름)
    dist: float                  # 원래 점에서 길 가운데 선까지 거리 (m)
    x: float                     # 길 가운데 선 위로 옮긴 점 (map, m)
    y: float
    cross: float = 0.0           # 원래 점이 가운데 선의 왼쪽(+)/오른쪽(-)으로 벗어난 거리 (m)


@dataclass
class Move:
    """갈림길에서 (들어온 간선 → 나갈 간선) 일 때의 동작."""
    turn: str
    follow: str
    follow_dist: float


@dataclass
class Junction:
    name: str
    x: float
    y: float
    radius: float
    moves: dict                  # {(from_edge, to_edge): Move}


@dataclass
class Leg:
    """경로의 한 구간: 간선 하나를 dir 방향으로 s_from → s_to 까지."""
    edge: str
    dir: int
    s_from: float
    s_to: float
    route_s0: float              # 이 구간이 경로에서 시작하는 위치 (m)

    @property
    def length(self):
        return abs(self.s_to - self.s_from)


@dataclass
class Maneuver:
    """갈림길 동작 (LaneManeuver 와 같은 필드)."""
    node_id: str
    s_at: float
    turn: str
    follow: str
    follow_dist: float


@dataclass
class Route:
    legs: list
    points: np.ndarray           # 경로 꺾은선 꼭짓점 (N x 2, map). 첫 점 = 시작, 마지막 점 = 목표
    length: float
    maneuvers: list
    uturn: bool                  # 출발 전에 제자리 U턴이 필요한가

    def locate_s(self, s):
        """경로 위 거리 s → (간선 이름, 간선 기준 s)."""
        s = min(max(s, 0.0), self.length)
        for lg in self.legs:
            if s <= lg.route_s0 + lg.length + EPS:
                return lg.edge, lg.s_from + lg.dir * (s - lg.route_s0)
        lg = self.legs[-1]
        return lg.edge, lg.s_to

    def s_of(self, edge, s_edge):
        """간선 위 위치 → 경로 위 거리 (처음 지나는 곳). 경로가 지나지 않으면 None."""
        for lg in self.legs:
            lo, hi = sorted((lg.s_from, lg.s_to))
            if lg.edge == edge and lo - EPS <= s_edge <= hi + EPS:
                return lg.route_s0 + abs(s_edge - lg.s_from)
        return None


class Edge:
    def __init__(self, name, point_names, pts, oneway):
        self.name = name
        self.names = list(point_names)
        self.oneway = bool(oneway)
        self.pts = np.asarray(pts, float)
        seg = np.diff(self.pts, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.cum[-1])
        self.node_a, self.node_b = self.names[0], self.names[-1]

    def allows(self, d):
        return d == 1 or not self.oneway

    def _seg(self, s):
        return min(max(int(np.searchsorted(self.cum, s, side='right')) - 1, 0), len(self.seg_len) - 1)

    def point_at(self, s):
        s = min(max(s, 0.0), self.length)
        i = self._seg(s)
        return self.pts[i] + (self.pts[i + 1] - self.pts[i]) * ((s - self.cum[i]) / self.seg_len[i])

    def tangent_at(self, s):
        i = self._seg(min(max(s, 0.0), self.length))
        return (self.pts[i + 1] - self.pts[i]) / self.seg_len[i]

    def project(self, p):
        """점 p → (s, 가운데 선까지 거리, 왼쪽(+) 벗어남)."""
        a, b = self.pts[:-1], self.pts[1:]
        ab = b - a
        t = np.clip(((p - a) * ab).sum(axis=1) / (self.seg_len ** 2), 0.0, 1.0)
        q = a + ab * t[:, None]
        d = np.hypot(p[0] - q[:, 0], p[1] - q[:, 1])
        i = int(np.argmin(d))
        u = ab[i] / self.seg_len[i]
        cross = u[0] * (p[1] - q[i][1]) - u[1] * (p[0] - q[i][0])
        return float(self.cum[i] + t[i] * self.seg_len[i]), float(d[i]), float(cross)

    def sub_points(self, s0, s1):
        """s0 → s1 사이 꺾은선 (s0 > s1 이면 거꾸로)."""
        lo, hi = sorted((s0, s1))
        inner = [self.pts[i] for i in range(len(self.pts)) if lo + EPS < self.cum[i] < hi - EPS]
        out = [self.point_at(lo)] + inner + [self.point_at(hi)]
        return out if s0 <= s1 else out[::-1]


class Course:
    def __init__(self, data, base_dir='.'):
        problems = []
        data = data or {}
        self.map_name = data.get('map', '')
        self.params = dict(DEFAULT_PARAMS)
        for k, v in (data.get('params') or {}).items():
            if k not in DEFAULT_PARAMS:
                problems.append(f'params.{k}: 모르는 설정')
            else:
                self.params[k] = float(v)

        self.points = {}
        for name, v in (data.get('points') or {}).items():
            self._add_point(name, v, 'estimate', problems)
        self.measured_file = ''
        if data.get('measured'):
            self.measured_file = os.path.join(base_dir, data['measured'])
            if os.path.exists(self.measured_file):
                with open(self.measured_file, encoding='utf-8') as f:
                    measured = (yaml.safe_load(f) or {}).get('points') or {}
                for name, v in measured.items():
                    if name not in self.points:
                        problems.append(f'실측 파일의 점 {name}: 코스 설정에 없는 이름')
                    else:
                        self._add_point(name, v, 'measured', problems)

        self.edges = {}
        for name, v in (data.get('edges') or {}).items():
            names = list((v or {}).get('points') or [])
            missing = [n for n in names if n not in self.points]
            if len(names) < 2 or missing:
                problems.append(f'간선 {name}: 점이 2개 이상 필요하고 모두 points 에 있어야 함 (없는 점: {missing})')
                continue
            pts = [(self.points[n].x, self.points[n].y) for n in names]
            if any(math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-3 for a, b in zip(pts, pts[1:])):
                problems.append(f'간선 {name}: 연속한 두 점이 같은 위치')
                continue
            e = Edge(name, names, pts, (v or {}).get('oneway', False))
            if e.node_a == e.node_b and not e.oneway:
                problems.append(f'간선 {name}: 처음과 끝이 같은(한 바퀴 도는) 간선은 일방통행이어야 함')
            self.edges[name] = e

        self.junctions = {}
        for name, v in (data.get('junctions') or {}).items():
            self._add_junction(name, v or {}, problems)
        self._check_nodes(problems)

        self.waits = {}
        for name, v in (data.get('waits') or {}).items():
            self._add_wait(name, v or {}, problems)
        self.crosswalks = {}
        for name, v in (data.get('crosswalks') or {}).items():
            p = self.points.get((v or {}).get('point'))
            if p is None:
                problems.append(f'횡단보도 {name}: point 가 points 에 없음')
            else:
                self.crosswalks[name] = (p.x, p.y, float((v or {}).get('radius', 0.1)))
        if problems:
            raise CourseError(problems)

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return cls(data, base_dir=os.path.dirname(os.path.abspath(path)))

    # ---------- 읽기 / 검사 ----------
    def _add_point(self, name, v, default_source, problems):
        if not isinstance(v, dict) or 'x' not in v or 'y' not in v:
            problems.append(f'점 {name}: x, y 가 필요함')
            return
        src = v.get('source', default_source)
        if src not in SOURCES:
            problems.append(f'점 {name}: source 는 estimate 또는 measured')
        yaw = v.get('yaw')
        self.points[name] = Point(name, float(v['x']), float(v['y']), None if yaw is None else float(yaw),
                                  src, str(v.get('stamp', '')))

    def _add_junction(self, name, v, problems):
        p = self.points.get(name)
        if p is None:
            problems.append(f'갈림길 {name}: 같은 이름의 점이 points 에 없음')
            return
        moves = {}
        for i, m in enumerate(v.get('moves') or []):
            frm, to = m.get('from'), m.get('to')
            where = f'갈림길 {name} moves[{i}]'
            if frm not in self.edges or to not in self.edges:
                problems.append(f'{where}: 없는 간선 ({frm} → {to})')
                continue
            if len(self._arrive_dirs(frm, name)) != 1 or len(self._depart_dirs(to, name)) != 1:
                problems.append(f'{where}: {frm} 로 들어오거나 {to} 로 나가는 방향이 하나로 정해지지 않음')
                continue
            if m.get('turn') not in TURNS or m.get('follow') not in FOLLOWS or float(m.get('follow_dist', 0)) <= 0:
                problems.append(f'{where}: turn 은 {TURNS}, follow 는 {FOLLOWS}, follow_dist > 0 이어야 함')
                continue
            moves[(frm, to)] = Move(m['turn'], m['follow'], float(m['follow_dist']))
        self.junctions[name] = Junction(name, p.x, p.y, float(v.get('radius', 0.2)), moves)

    def _arrive_dirs(self, edge, node):
        e = self.edges[edge]
        return [d for d, end in ((1, e.node_b), (-1, e.node_a)) if end == node and e.allows(d)]

    def _depart_dirs(self, edge, node):
        e = self.edges[edge]
        return [d for d, end in ((1, e.node_a), (-1, e.node_b)) if end == node and e.allows(d)]

    def _check_nodes(self, problems):
        ends = {}
        for e in self.edges.values():
            for n in (e.node_a, e.node_b):
                ends[n] = ends.get(n, 0) + 1
        for n, cnt in ends.items():
            if cnt >= 2 and n not in self.junctions:
                problems.append(f'점 {n}: 간선 끝이 {cnt}개 모이므로 junctions 에 있어야 함')
        for j in self.junctions.values():
            if ends.get(j.name, 0) < 2:
                problems.append(f'갈림길 {j.name}: 간선 끝이 2개 이상 모여야 함')
                continue
            for e in self.edges.values():          # 들어오는 길마다 나갈 동작이, 나가는 길마다 들어올 동작이 있어야 한다
                if self._arrive_dirs(e.name, j.name) and not any(f == e.name for f, _ in j.moves):
                    problems.append(f'갈림길 {j.name}: {e.name} 로 들어온 로봇이 나갈 moves 가 없음')
                if self._depart_dirs(e.name, j.name) and not any(t == e.name for _, t in j.moves):
                    problems.append(f'갈림길 {j.name}: {e.name} 로 나가는 moves 가 없음')

    def _add_wait(self, name, v, problems):
        p = self.points.get(v.get('point'))
        if p is None:
            problems.append(f'대기 지점 {name}: point 가 points 에 없음')
            return
        loc = self.locate(p.x, p.y, max_dist=WAIT_ON_ROAD)
        if loc is None:
            problems.append(f'대기 지점 {name}: 길 가운데 선에서 {WAIT_ON_ROAD} m 안에 있어야 함')
            return
        for j in self.junctions.values():
            if math.hypot(p.x - j.x, p.y - j.y) <= j.radius:
                problems.append(f'대기 지점 {name}: 갈림길 {j.name} 구역(반지름 {j.radius} m) 안에 있음')
        self.waits[name] = loc

    def estimate_points(self):
        return [p.name for p in self.points.values() if p.source == 'estimate']

    # ---------- 위치 ----------
    def locate(self, x, y, yaw=None, max_dist=None):
        """점 (x, y) 에서 가장 가까운 길 위 위치. yaw 를 주면 방향이 맞는 길을 우선하고 dir 을 정한다."""
        max_dist = self.params['snap_max'] if max_dist is None else max_dist
        p = np.array([x, y], float)
        best = None
        for e in self.edges.values():
            s, d, cross = e.project(p)
            if d > max_dist:
                continue
            if yaw is None:
                score, dr = d, 0
            else:
                t = e.tangent_at(s)
                c = math.cos(yaw) * t[0] + math.sin(yaw) * t[1]
                dr = (1 if c >= 0 else -1) if abs(c) >= ALIGN_MIN else 0
                score = d + 0.1 * (1.0 - abs(c))
            if best is None or score < best[0]:
                q = e.point_at(s)
                best = (score, Location(e.name, s, dr, d, float(q[0]), float(q[1]), cross))
        return None if best is None else best[1]

    def location_at(self, edge, s):
        e = self.edges[edge]
        q = e.point_at(s)
        return Location(edge, min(max(s, 0.0), e.length), 0, 0.0, float(q[0]), float(q[1]))

    def distance_to_road(self, x, y):
        p = np.array([x, y], float)
        return min(e.project(p)[1] for e in self.edges.values())

    # ---------- 목표 ----------
    def zones(self, edge):
        """간선 위 목표 금지 구간 [(lo, hi, 이름)] (겹치면 합침)."""
        e = self.edges[edge]
        raw = []
        for j in self.junctions.values():
            if e.node_a == j.name:
                raw.append((0.0, j.radius, f'갈림길 {j.name}'))
            if e.node_b == j.name:
                raw.append((e.length - j.radius, e.length, f'갈림길 {j.name}'))
        c = self.params['dead_end_clearance']
        if e.node_a not in self.junctions:
            raw.append((0.0, c, '막다른 끝'))
        if e.node_b not in self.junctions:
            raw.append((e.length - c, e.length, '막다른 끝'))
        k = self.params['wait_keepout']
        for name, loc in self.waits.items():
            if loc.edge == edge:
                raw.append((loc.s - k, loc.s + k, f'대기 지점 {name}'))
        for name, (x, y, r) in self.crosswalks.items():
            s, d, _ = e.project(np.array([x, y]))
            if d <= self.params['lane_width']:
                raw.append((s - r, s + r, f'횡단보도 {name}'))
        merged = []
        for lo, hi, label in sorted(raw):
            lo, hi = max(lo, 0.0), min(hi, e.length)
            if merged and lo <= merged[-1][1] + EPS:
                plo, phi, plabel = merged[-1]
                merged[-1] = (plo, max(phi, hi), plabel if label in plabel else f'{plabel}·{label}')
            else:
                merged.append((lo, hi, label))
        return merged

    @staticmethod
    def _zone_at(zones, s):
        for lo, hi, label in zones:
            if lo - EPS <= s <= hi + EPS:
                return lo, hi, label
        return None

    def snap_goal(self, x, y):
        """클릭한 점 → (길 위 목표 Location 또는 None, 설명).
        가장 가까운 길 위 점이 금지 구역(갈림길·대기 지점·횡단보도·막다른 끝) 안이면,
        근처 길들의 구역 경계 중 클릭한 점에서 가장 가까운 곳으로 옮긴다 (구역 밖으로만 옮기므로 갈림길에서 멀어진다)."""
        p = np.array([x, y], float)
        near = []
        for e in self.edges.values():
            s, d, _ = e.project(p)
            if d <= self.params['snap_max']:
                near.append((d, e.name, s))
        if not near:
            return None, f'길에서 {self.distance_to_road(x, y):.2f} m 떨어져 있음 (최대 {self.params["snap_max"]:.2f} m)'
        near.sort()
        d0, edge0, s0 = near[0]
        hit = self._zone_at(self.zones(edge0), s0)
        if hit is None:
            loc = self.location_at(edge0, s0)
            loc.dist = d0
            return loc, ''
        cands = []
        for _, name, s in near:
            e, zones = self.edges[name], self.zones(name)
            z = self._zone_at(zones, s)
            for c in ([s] if z is None else [z[0] - 0.01, z[1] + 0.01]):
                if 0.0 <= c <= e.length and self._zone_at(zones, c) is None:
                    q = e.point_at(c)
                    cands.append((math.hypot(q[0] - x, q[1] - y), name, c))
        if not cands:
            return None, f'{hit[2]} 안이라 목표를 둘 수 없음'
        dist, name, c = min(cands)
        return self.location_at(name, c), f'{hit[2]} 안이라 {dist:.2f} m 떨어진 곳으로 옮김'

    def can_uturn(self, x, y, clearance_fn=None):
        """(x, y) 에서 제자리 U턴을 해도 되는가 → (bool, 이유). clearance_fn(x, y) 는 벽까지 거리(m)."""
        for j in self.junctions.values():
            if math.hypot(x - j.x, y - j.y) <= j.radius:
                return False, f'갈림길 {j.name} 구역 안'
        if clearance_fn is not None:
            c = float(clearance_fn(x, y))
            if c < self.params['uturn_clearance']:
                return False, f'벽까지 여유 {c:.2f} m (필요 {self.params["uturn_clearance"]:.2f} m)'
        return True, ''

    # ---------- 경로 ----------
    def plan(self, start, goal, allow_uturn=False):
        """start(Location, dir 필요) → goal(Location) 경로. 일방통행·막다른 끝을 지키는 가장 짧은 길을 고른다."""
        if start.dir not in (1, -1):
            raise PlanError('로봇 방향을 알 수 없음 (길과 거의 수직)')
        order = itertools.count()
        heap = []

        def push(cost, item):
            heapq.heappush(heap, (cost, next(order), item))

        e0 = self.edges[start.edge]
        for d, uturn in ((start.dir, False), (-start.dir, True)):
            if (uturn and not allow_uturn) or not e0.allows(d):
                continue
            pen = self.params['uturn_penalty'] if uturn else 0.0
            if goal.edge == e0.name and (goal.s - start.s) * d >= -EPS:
                push(pen + abs(goal.s - start.s), ('done', [(e0.name, d, start.s, goal.s)], [], uturn))
            s_end = e0.length if d > 0 else 0.0
            node = e0.node_b if d > 0 else e0.node_a
            push(pen + abs(s_end - start.s), ('node', node, (e0.name, d), [(e0.name, d, start.s, s_end)], [], uturn))

        visited = set()
        while heap:
            cost, _, item = heapq.heappop(heap)
            if item[0] == 'done':
                return self._build_route(item[1], item[2], item[3])
            _, node, arrived, legs, moves, uturn = item
            if (node, arrived) in visited:
                continue
            visited.add((node, arrived))
            j = self.junctions.get(node)
            if j is None:
                continue                                   # 막다른 끝: 더 갈 수 없다
            for (frm, to), mv in j.moves.items():
                if frm != arrived[0] or self._arrive_dirs(frm, node) != [arrived[1]]:
                    continue
                d = self._depart_dirs(to, node)[0]
                e = self.edges[to]
                s_start = 0.0 if d > 0 else e.length
                rec = (node, mv)
                if goal.edge == to and (goal.s - s_start) * d >= -EPS:
                    done = ('done', legs + [(to, d, s_start, goal.s)], moves + [rec], uturn)
                    push(cost + abs(goal.s - s_start), done)
                s_end = e.length if d > 0 else 0.0
                nxt = e.node_b if d > 0 else e.node_a
                push(cost + e.length, ('node', nxt, (to, d), legs + [(to, d, s_start, s_end)], moves + [rec], uturn))
        if not allow_uturn:
            raise PlanError('U턴 없이는 갈 수 있는 경로가 없음 (이 위치에서는 U턴이 허가되지 않음)')
        raise PlanError('갈 수 있는 경로가 없음')

    def _build_route(self, raw_legs, moves, uturn):
        legs, pts, route_s = [], [], 0.0
        for edge, d, s_from, s_to in raw_legs:
            lg = Leg(edge, d, s_from, s_to, route_s)
            legs.append(lg)
            for q in self.edges[edge].sub_points(s_from, s_to):
                if not pts or math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 1e-9:
                    pts.append(np.asarray(q, float))
            route_s += lg.length
        if len(pts) == 1:
            pts.append(pts[0].copy())
        maneuvers = [Maneuver(node, legs[i + 1].route_s0, mv.turn, mv.follow, mv.follow_dist)
                     for i, (node, mv) in enumerate(moves)]
        return Route(legs, np.array(pts), route_s, maneuvers, uturn)
