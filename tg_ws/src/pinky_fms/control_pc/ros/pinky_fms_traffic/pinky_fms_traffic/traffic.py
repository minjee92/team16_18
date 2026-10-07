"""계획 경로 기반 충돌·교착 방지 (2대 이상 일반화 가능한 쌍별 규칙).

고정 대기 장소가 없다. 매 주기마다 각 로봇의 '남은 계획 경로'를 서로 겹쳐 본다.
  1) 두 경로가 (공간적으로) R_CONF 이내로 가까워지고 (시간적으로) 비슷한 때 지나가면 충돌 예정.
  2) 먼저 도착하는 쪽(ETA 작은 쪽)이 우선. 진 쪽은
       - 아직 충돌 구간에 들어가기 전이면  → 충돌 구간 앞(좁은 통로면 통로 입구 앞)에서 HOLD
       - 이미 상대 경로 위에 있으면        → 상대의 남은 경로에서 벗어난 가장 가까운 칸으로 YIELD (경로 위 비켜서기)
  3) 상대가 지나가 충돌이 사라지면 다음 주기에 자동으로 풀린다 (상태를 따로 기억하지 않는다).
좁은 통로(여유 NARROW_CLEAR 미만)는 한 대씩만: 시간 간격을 크게 잡아 통로 안에서 마주치지 않게 한다.
"""
import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from .planner import NEI, passable

R_BODY = 0.095            # 제자리 회전 시 몸체 반경 (뒤쪽 모서리, URDF 충돌 메시 기준)
MARGIN = 0.06             # 위치추정·주행 오차 여유
R_CONF = 2 * R_BODY + MARGIN * 0 + 0.05    # 두 로봇 중심이 이보다 가까우면 충돌 위험
T_GAP = 3.0               # 같은 곳을 이 시간(s) 안에 지나가면 충돌 예정
T_GAP_NARROW = 8.0        # 좁은 통로는 한 대씩
STAND_CLEAR = 0.17        # 비켜서서 기다릴 수 있는 최소 여유 (실제 Nav2 가 계획·회전할 수 있는 거리. 0.10 m 에서는 collision ahead 로 실패했다)
RETREAT_MAX = 2.0         # 교착 때 후진으로 물러날 수 있는 최대 거리 (m)
YIELD_HORIZON = 6.0       # 상대가 충돌 지점에 이 시간(s) 안에 오지 않으면 아직 비켜서지 않는다
LANE_SINGLE_W = 0.42      # 통로 폭이 이보다 좁으면 1차선(두 대가 나란히 설 수 없음)
LANE_MARGINAL_W = 0.64    # 이보다 좁으면 한계 폭(양쪽이 같은 쪽으로 치우쳐야 지나감), 이상이면 2차선
LANE_BUFFER = 0.24        # 1차선 입구에서 이만큼은 떨어져서 기다린다 (상대가 빠져나오는 길을 막지 않게)
NARROW_CLEAR = 0.20       # 이 여유 미만이면 좁은 통로
SPEED_UNCERTAINTY = 0.35  # 실제 속도가 계획보다 느릴 수 있는 비율 (도착 시각 오차는 ETA 에 비례)
BACKOFF = 0.20            # 충돌 구간 앞에서 멈출 때 더 물러서는 거리 (m)


@dataclass
class Agent:
    id: str
    pos: np.ndarray
    plan: np.ndarray                  # 현재 위치부터의 남은 경로 (ds 간격)
    ds: float = 0.02
    v: float = 0.2
    parked: bool = False              # 목표 도착해 서 있음 (남은 경로 없음)
    yielding: bool = False            # 비켜서는 중 (우선권)
    trail: np.ndarray = None          # 지나온 위치 (오래된 것 → 최근, ds 간격). 후진으로 길을 터 줄 때 되돌아갈 길
    backing: bool = False             # 후진으로 길을 터 주는 중
    yaw: float = None                 # 로봇이 보는 방향 (똑바로 후진할 때 뒤쪽 공간 계산용)
    fixed: bool = False               # 관제가 움직일 수 없는 로봇 (차선 주행): 명령을 따르지 않고, 마주치면 항상 우선 (encounter 방식만)


@dataclass
class Command:
    kind: str                         # GO | HOLD | YIELD | BACKUP(후진으로 path 를 따라 물러난다)
    hold_index: int = 0               # HOLD: 이 인덱스까지만 진행
    path: np.ndarray = None           # YIELD: 비켜설 경로
    why: str = ''


class TrafficManager:
    def __init__(self, gm, r_conf=R_CONF, r_conf_parked=0.30, yield_sep=None):
        self.gm = gm
        self.r_conf = r_conf
        self.r_conf_parked = min(r_conf_parked, r_conf)   # 서 있는 로봇과는 더 가까이 지나가도 된다 (움직이는 로봇끼리보다 오차 여유를 덜 둔다)
        # 비켜선 자리와 상대 경로 사이 최소 거리. 충돌 판정 거리보다 작으면 비켜선 뒤에도 '막고 있다'로 판정돼 반복하므로 그 이상으로만 둔다.
        # 실물 Nav2 는 계획 경로보다 코너를 더 깎아 지나가므로(2026-10-06 실물: 비켜선 자리에서 8 cm 옆을 지나감) 여유를 더 줄 수 있다.
        self.yield_sep = max(r_conf, yield_sep if yield_sep is not None else r_conf)
        self.straight_tries = {}                      # (id,id) -> 똑바로 후진한 횟수 (교착이 안 풀릴 때 마지막 수단, 3번까지)
        self.last_bay = {}                            # 로봇 -> 직전에 고른 passing bay (같은 곳을 유지해 목표가 계속 바뀌는 것을 막는다)
        # 통로 폭: 가운데 선(여유가 국소 최대인 칸)의 여유×2 를 각 칸이 가장 가까운 가운데 선에서 물려받는다.
        # 열린 방의 벽가는 방 가운데의 넓은 값을, 복도는 좁은 값을 얻는다 (좌표를 쓰지 않으므로 맵이 바뀌어도 동작).
        from scipy import ndimage as ndi
        c = ndi.gaussian_filter(gm.clear, 1.0)
        gy, gx = np.gradient(c)
        gn = np.hypot(gx, gy) + 1e-9
        rr, cc = np.mgrid[0:gm.H, 0:gm.W].astype(float)
        step = 1.2
        c_fwd = ndi.map_coordinates(c, [rr + gy / gn * step, cc + gx / gn * step], order=1, mode='nearest')
        c_bwd = ndi.map_coordinates(c, [rr - gy / gn * step, cc - gx / gn * step], order=1, mode='nearest')
        # 가운데 선: 여유가 늘어나는 방향(기울기 방향)으로도, 반대로도 더 커지지 않는 칸 (벽과 나란한 평평한 곳은 기울기 방향으로 커지므로 제외)
        ridge = gm.free & (gm.clear > 0.04) & (c >= c_fwd - 1e-4) & (c >= c_bwd - 1e-4)
        idx = ndi.distance_transform_edt(~ridge, return_distances=False, return_indices=True)
        self.width = 2 * gm.clear[idx[0], idx[1]]
        self.lane = np.where(self.width < LANE_SINGLE_W, 2, np.where(self.width < LANE_MARGINAL_W, 1, 0))   # 2=1차선 1=한계 0=2차선 이상
        self.narrow = gm.free & (self.lane == 2)
        self.dnarrow = ndi.distance_transform_edt(~self.narrow) * gm.res if self.narrow.any() else np.full((gm.H, gm.W), 9.0)
        self.winner = {}        # (id, id) -> 우선권 가진 로봇 (충돌이 사라질 때까지 유지: 서로 번갈아 양보하다 멈추는 것을 막는다)
        self.clear_ticks = {}
        self._dgoal = {}

    def reset(self):
        self.winner.clear()
        self.clear_ticks.clear()
        self.last_bay.clear()
        self.straight_tries.clear()

    def _cells(self, pts):
        pts = np.atleast_2d(pts)
        r = np.clip(((pts[:, 1] - self.gm.oy) / self.gm.res).astype(int), 0, self.gm.H - 1)
        c = np.clip(((pts[:, 0] - self.gm.ox) / self.gm.res).astype(int), 0, self.gm.W - 1)
        return r, c

    def _narrow_at(self, pts):
        return self.narrow[self._cells(pts)]

    def lane_class_at(self, pts):
        return self.lane[self._cells(pts)]

    def conflict(self, a: Agent, b: Agent):
        """(a 쪽 첫 충돌 인덱스, b 쪽 첫 충돌 인덱스) 또는 None"""
        A, B = a.plan, b.plan
        d = np.hypot(A[:, None, 0] - B[None, :, 0], A[:, None, 1] - B[None, :, 1])
        still = a.parked or b.parked or len(A) <= 1 or len(B) <= 1
        close = d < (self.r_conf_parked if still else self.r_conf)
        if not close.any():
            return None
        ta = np.arange(len(A)) * a.ds / a.v
        tb = np.arange(len(B)) * b.ds / b.v
        dt = np.abs(ta[:, None] - tb[None, :])
        na, nb = self._narrow_at(A), self._narrow_at(B)
        base = np.where(na[:, None] & nb[None, :], T_GAP_NARROW, T_GAP)
        gap = base + SPEED_UNCERTAINTY * np.maximum(ta[:, None], tb[None, :])     # 멀리 있는 일수록 도착 시각이 틀어진다
        ok = dt < gap
        if a.parked or b.parked or len(A) <= 1 or len(B) <= 1:     # 서 있는 로봇은 시간과 상관없이 막는다
            ok = np.ones_like(ok)
        hit = close & ok
        if not hit.any():
            return None
        ia = int(np.nonzero(hit.any(axis=1))[0][0])
        ib = int(np.nonzero(hit.any(axis=0))[0][0])
        return ia, ib

    def _dist_to(self, goal):
        """goal 에서 각 칸까지의 이동 거리(m). 같은 목표는 캐시한다."""
        gm = self.gm
        key = gm.cell(*goal)
        if key in self._dgoal:
            return self._dgoal[key]
        mask = passable(gm)
        d = np.full((gm.H, gm.W), 1e9)
        if not mask[key]:
            from .planner import snap
            key = snap(gm, mask, goal)
        d[key] = 0.0
        pq = [(0.0, key)]
        while pq:
            dc, cur = heapq.heappop(pq)
            if dc > d[cur]:
                continue
            for dr, dcc in NEI:
                n = (cur[0] + dr, cur[1] + dcc)
                if gm.inside(*n) and mask[n]:
                    nd = dc + math.hypot(dr, dcc) * gm.res
                    if nd < d[n]:
                        d[n] = nd
                        heapq.heappush(pq, (nd, n))
        self._dgoal[gm.cell(*goal)] = d
        return d

    def _eta(self, a: Agent, idx):
        if a.yielding or a.backing:
            return -1.0
        if a.parked or len(a.plan) <= 1:
            return 1e9
        return idx * a.ds / a.v

    def decide(self, agents):
        cmds = {a.id: Command('GO') for a in agents}
        for a in agents:                                   # 비켜서는 중: 상대가 움직인 만큼 비켜서는 길을 다시 계산
            if not a.yielding or a.backing:
                continue
            for o in agents:
                if o.id != a.id and self.conflict(a, o) is not None:
                    cmds[a.id] = self._yield(a, o, agents)
                    break
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                a, b = agents[i], agents[j]
                key = (a.id, b.id)
                if (a.yielding or b.yielding) and not (a.backing or b.backing) and not (a.yielding and b.yielding):
                    # 한쪽이 비켜서는 중인데 둘이 가까우면(먼저 빠져나갈 시간이 없다) 다른 쪽은 비킬 때까지 기다린다
                    y, o = (a, b) if a.yielding else (b, a)
                    if np.hypot(*(a.pos - b.pos)) < self.r_conf + 0.20 and cmds[o.id].kind == 'GO' and self.conflict(a, b) is not None:
                        cmds[o.id] = Command('HOLD', 0, why=f'{y.id} 가 가까이서 비키는 중 대기')
                        continue
                if a.backing or b.backing:                 # 한쪽이 후진으로 길을 트는 중: 다른 쪽은 제자리에서 기다린다
                    back, other = (a, b) if a.backing else (b, a)
                    if not other.backing and cmds[other.id].kind == 'GO' and self.conflict(a, b) is not None:
                        cmds[other.id] = Command('HOLD', 0, why=f'{back.id} 가 후진으로 길을 트는 중 대기')
                    continue
                c = self.conflict(a, b)
                if c is None:
                    n = self.clear_ticks.get(key, 0) + 1
                    self.clear_ticks[key] = n
                    if n >= 2:                       # 2주기 연속 충돌이 없으면 우선권 해제
                        self.winner.pop(key, None)
                        self.straight_tries.pop(key, None)
                    continue
                self.clear_ticks[key] = 0
                ea, eb = self._eta(a, c[0]), self._eta(b, c[1])
                if key in self.winner:
                    wid = self.winner[key]
                elif self._narrow_at(a.pos[None, :])[0] and self._narrow_at(b.pos[None, :])[0] and not (a.parked or b.parked):
                    # 둘 다 1차선 통로 안: 통로 밖으로 더 짧게 되돌아 나갈 수 있는 쪽이 물러난다 (대기 시간 최소). 같으면 id 로.
                    ca, cb = self._yield(a, b, agents), self._yield(b, a, agents)
                    la = len(ca.path) if ca.kind == 'YIELD' else 1e9
                    lb = len(cb.path) if cb.kind == 'YIELD' else 1e9
                    wid = b.id if (la < lb or (la == lb and a.id > b.id)) else a.id     # 물러나는 쪽이 진다 → 이긴 쪽 = 상대
                elif abs(ea - eb) < 0.5:             # 비슷하면 id 가 앞선 쪽 우선
                    wid = a.id
                else:
                    wid = a.id if ea < eb else b.id
                if (a.parked or len(a.plan) <= 1) != (b.parked or len(b.plan) <= 1):     # 서 있는 로봇은 이긴 적이 있어도 움직이는 로봇에게 양보
                    wid = b.id if (a.parked or len(a.plan) <= 1) else a.id
                self.winner[key] = wid
                win, lose, il = (a, b, c[1]) if wid == a.id else (b, a, c[0])
                iw0 = c[0] if win is a else c[1]
                etaw = self._eta(win, iw0)
                if (lose.parked or len(lose.plan) <= 1) and not (win.parked or len(win.plan) <= 1):
                    cmd = self._yield(lose, win, agents) if etaw <= YIELD_HORIZON else Command('GO')
                else:
                    cmd = self._hold_or_yield(lose, win, il, agents, etaw)
                if cmd.kind == 'HOLD' and cmd.why.startswith('비켜설 곳 없음'):
                    # 진 쪽이 비켜설 곳이 없으면: (1) 이긴 쪽을 잠깐 세워 두고 진 쪽이 뒤로 빠져 비킨다
                    rl = self._retreat(lose, win, agents)
                    if rl is not None:
                        self.winner[key] = win.id
                        cmds[lose.id] = Command('BACKUP', path=rl, why=f'{win.id} 와 마주침: 뒤로 빠져 {len(rl) * lose.ds:.2f} m 이동해 길 비키기')
                        if cmds[win.id].kind == 'GO':
                            cmds[win.id] = Command('HOLD', 0, why=f'{lose.id} 가 길을 트는 중 대기')
                        continue
                    # (2) 역할을 바꿔 이긴 쪽이 비켜 본다. (3) 그것도 안 되면 둘 중 덜 움직이는 쪽이 뒤로 빠진다. (4) 모두 안 되면 정지
                    iw = c[0] if win is a else c[1]
                    alt = self._hold_or_yield(win, lose, iw, agents)
                    if alt.kind == 'YIELD':
                        self.winner[key] = lose.id
                        if cmds[win.id].kind == 'GO':
                            cmds[win.id] = alt
                        cmd = Command('HOLD', 0, why=f'{win.id} 가 비키는 중 대기')
                    else:
                        # 둘 다 옆으로 비킬 곳이 없다: 지나온 길로 덜 물러나도 되는 쪽이 후진해서 상대 경로 밖으로 빠진다
                        ra, rb = self._retreat(lose, win, agents), self._retreat(win, lose, agents)
                        la = len(ra) if ra is not None else 1e9
                        lb = len(rb) if rb is not None else 1e9
                        if min(la, lb) < 1e9:
                            back, stay, path = (lose, win, ra) if la <= lb else (win, lose, rb)
                            self.winner[key] = stay.id
                            cmds[back.id] = Command('BACKUP', path=path, why=f'{stay.id} 와 마주침: 뒤로 빠져 {len(path) * back.ds:.2f} m 이동해 길 비키기')
                            if cmds[stay.id].kind == 'GO':
                                cmds[stay.id] = Command('HOLD', 0, why=f'{back.id} 가 후진으로 길을 트는 중 대기')
                            continue
                        # 그래도 길이 없으면: 뒤쪽 공간이 더 넓은 로봇이 똑바로 0.15 m 후진 (상대와 멀어지는 쪽만). 3번까지 반복하며 다시 판단
                        tries = self.straight_tries.get(key, 0)
                        if tries < 3:
                            fa, fb = self._back_free(lose, win, agents), self._back_free(win, lose, agents)
                            if max(fa, fb) >= 0.08:
                                back, stay = (lose, win) if fa >= fb else (win, lose)
                                self.straight_tries[key] = tries + 1
                                self.winner[key] = stay.id
                                d = min(0.15, max(fa, fb))
                                cmds[back.id] = Command('BACKUP_STRAIGHT', hold_index=int(round(d * 100)),
                                                        why=f'{stay.id} 와 붙어 비킬 길이 없음: 똑바로 {d:.2f} m 후진 ({tries + 1}/3)')
                                if cmds[stay.id].kind == 'GO':
                                    cmds[stay.id] = Command('HOLD', 0, why=f'{back.id} 가 후진하는 중 대기')
                                continue
                        alt.why = f'{lose.id} 가 비킬 곳이 없어 대기'
                        if cmds[win.id].kind == 'GO':
                            cmds[win.id] = alt
                        cmd = Command('HOLD', 0, why='둘 다 비킬 곳 없음: 제자리 대기')
                if cmd.kind == 'YIELD' and cmd.path is not None and len(cmd.path) > 3 and np.hypot(*(lose.pos - win.pos)) < self.r_conf + 0.20 and cmds[win.id].kind == 'GO':
                    cmds[win.id] = Command('HOLD', 0, why=f'{lose.id} 가 가까이서 비키는 중 대기')
                if lose.yielding:                  # 비켜서는 중인 로봇은 위에서 이미 갱신했다
                    continue
                prev = cmds[lose.id]
                if prev.kind == 'GO' or (cmd.kind == 'HOLD' and prev.kind == 'HOLD' and cmd.hold_index < prev.hold_index):
                    cmds[lose.id] = cmd
        return cmds

    def _hold_or_yield(self, lose, win, ic, agents, etaw=0.0):
        gm, ds = self.gm, lose.ds
        idx = ic - int(BACKOFF / ds) - int(self.r_conf / 2 / ds)
        while idx > 0 and self._narrow_at(lose.plan[idx:idx + 1])[0]:          # 좁은 통로 안이면 입구 앞까지
            idx -= 1
        idx -= int(0.10 / ds) if idx > 0 else 0
        while idx > 0 and np.min(np.hypot(*(win.plan - lose.plan[idx]).T)) < self.r_conf:
            idx -= 1
        while idx > 0 and self.dnarrow[self._cells(lose.plan[idx:idx + 1])][0] < LANE_BUFFER:     # 1차선 입구에서 완충 거리 이상 떨어져서 대기
            idx -= 1
        idx = max(idx, 0)
        if idx == 0:
            blocking = np.min(np.hypot(*(win.plan - lose.pos).T)) < self.yield_sep     # 비켜선 자리 기준과 같은 값 (다르면 이미 비켜선 로봇이 계속 막고 있다고 판정된다)
            if blocking:
                if etaw > YIELD_HORIZON and not lose.yielding:
                    return Command('GO')                  # 상대가 아직 멀다: 지금 비켜설 필요 없음
                return self._yield(lose, win, agents)
            return Command('HOLD', 0, why=f'{win.id} 통과 대기')
        return Command('HOLD', idx, why=f'{win.id} 통과 대기(충돌 구간 앞)')

    def _back_free(self, a, other, agents, max_d=0.30):
        """a 가 지금 방향 그대로 똑바로 후진할 수 있는 거리 (벽 여유 0.08 m, 다른 로봇 0.19 m). 후진하면 other 와 가까워지면 0."""
        if a.yaw is None:
            return 0.0
        back = -np.array([math.cos(a.yaw), math.sin(a.yaw)])
        if np.dot(back, a.pos - other.pos) < 0:          # 뒤쪽에 상대가 있다
            return 0.0
        others = [o.pos for o in agents if o.id != a.id]
        d = 0.0
        while d < max_d:
            p = a.pos + back * (d + 0.01)
            if self.gm.clearance_at(p)[0] < 0.08 or any(np.hypot(*(p - q)) < 2 * R_BODY for q in others):
                break
            d += 0.01
        return d

    def _retreat(self, a, other, agents):
        """교착 해소: other 가 제자리에서 기다린다고 보고, a 가 (필요하면 뒤로 빠졌다가) other 의 남은 경로 밖 자리까지 가는 경로.
        못 찾으면 None."""
        c = self._yield(a, other, agents, static=True)
        if c.kind != 'YIELD' or c.path is None or len(c.path) < 2:
            return None
        return c.path

    def _yield(self, lose, win, agents, static=False):
        """lose 가 서 있는 곳에서 win 의 남은 경로를 벗어난 가장 가까운 (좁지 않은) 칸으로"""
        gm = self.gm
        mask = passable(gm)
        dwin = gm.raster(win.plan)
        near_win_now = gm.raster(win.pos[None, :])
        others = [a.pos for a in agents if a.id not in (lose.id, win.id)]
        sep = self.r_conf_parked if (lose.parked or len(lose.plan) <= 1) else self.yield_sep
        goal_ok = mask & (dwin > sep) & (gm.clear >= STAND_CLEAR) & (self.dnarrow >= LANE_BUFFER)
        for p in others:
            goal_ok &= gm.raster(p[None, :]) > self.r_conf
        walk = mask & (near_win_now > R_BODY * 2 * 0.9)       # win 의 현재 위치 근처로는 지나가지 않는다
        s = gm.cell(*lose.pos)
        if not walk[s]:
            walk = walk.copy()
            walk[s] = True
        # win 이 앞으로 있을 위치(0.25초 간격)를 미리 계산: 비켜서는 길이 그 위치 가까이를 지나지 않게 한다
        step = max(1, int(0.25 * win.v / win.ds))
        wp = win.plan[::step] if not static else win.pos[None, :]      # static: 상대는 제자리에서 기다린다
        if len(wp) < 2:
            wp = np.vstack([wp, wp])
        keep = 0.85 * self.r_conf
        win_end = len(wp) * 0.25

        def hits_win(n, tl):
            k0, k1 = max(int((tl - 0.75) / 0.25), 0), int((tl + 0.75) / 0.25) + 1
            if k0 >= len(wp):                                  # win 이 경로 끝에 서 있게 된 뒤
                k0, k1 = len(wp) - 1, len(wp)
            seg = wp[k0:min(k1, len(wp))]
            w = gm.world(*n)
            return bool(np.any(np.hypot(seg[:, 0] - w[0], seg[:, 1] - w[1]) < keep))

        dgoal = self._dist_to(lose.plan[-1]) if len(lose.plan) > 1 else None
        escape = near_win_now[s] < max(0.85 * self.r_conf, R_BODY * 2 * 0.9) + 0.05     # 이미 상대와 붙어 있다
        pq = [(0.0, s)]
        dist = {s: 0.0}
        came = {}
        target, best = None, 1e18
        while pq:
            dcur, cur = heapq.heappop(pq)
            if dcur > dist.get(cur, 1e18):
                continue
            if dcur >= best:                                   # 더 멀리 가서 더 좋아질 수 없다
                break
            if goal_ok[cur]:
                # 최적 passing bay: (비켜서러 가는 길) + (거기서 원래 목표까지 가는 길) 이 가장 짧은 곳. 여유가 넉넉할수록 유리.
                total = dcur + (dgoal[cur] if dgoal is not None else 0.0) + 0.4 * max(0.0, 0.25 - gm.clear[cur])
                lb = self.last_bay.get(lose.id)
                if lb is not None and lb[1] == win.id and math.hypot(cur[0] - lb[0][0], cur[1] - lb[0][1]) * gm.res < 0.15:
                    total -= 0.8                           # 직전에 고른 곳 근처면 우대 (왔다 갔다 하지 않게)
                if total < best:
                    best, target = total, cur
            for dr, dc in NEI:
                n = (cur[0] + dr, cur[1] + dc)
                if not gm.inside(*n) or not mask[n]:
                    continue
                away = escape and near_win_now[n] > near_win_now[cur] + 1e-9     # 상대와 붙어 있을 때: 멀어지는 칸은 허용
                if not walk[n] and not away:
                    continue
                nd = dcur + math.hypot(dr, dc) * gm.res * (1 + 0.5 * math.exp(-10 * max(gm.clear[n] - 0.07, 0)))
                if n != s and not away and hits_win(n, nd / lose.v):
                    continue
                if nd < dist.get(n, 1e18):
                    dist[n] = nd
                    came[n] = cur
                    heapq.heappush(pq, (nd, n))
        if target is None:
            return Command('HOLD', 0, why=f'비켜설 곳 없음({win.id})')
        self.last_bay[lose.id] = (target, win.id)
        cells = [target]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        from .planner import resample
        path = resample(np.array([gm.world(*c) for c in cells[::-1]]), lose.ds)
        return Command('YIELD', path=path, why=f'{win.id} 경로에서 비켜서기')
