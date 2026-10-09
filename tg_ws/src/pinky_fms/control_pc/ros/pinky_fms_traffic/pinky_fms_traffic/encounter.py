"""마주침 기반 충돌 처리 (2026-10-07 사용자 사양). 미리 예측해서 피하지 않고, 실제로 마주쳤을 때만 개입한다.

  1) 주행: 각 로봇은 자기 Nav2 계획 경로로 주행한다. 관제는 지켜보기만 한다.
  2) 마주침: 두 로봇이 D_MEET 안 + 사이에 벽 없음 + 서로 가까워지는 중
            + 한쪽 몸체가 상대의 '남은 경로 앞 L_AHEAD' 점유 띠(±R_TUBE) 안
  3) 일시 정지: 둘 다 PAUSE 초 정지
  4) 우선권: 미션 없이 서 있는 로봇은 항상 양보. 아니면 상대 경로에서 비켜나기 쉬운(이동 거리가 짧은) 쪽이 양보,
            비슷하면 목표에 더 가까운 쪽이 우선
  5) 우선권 로봇: 마주친 순간의 경로를 고정하고, 양보 로봇 몸체가 그 경로 띠에서 빠지면 바로 출발
     양보 로봇: 우선권 로봇의 (남은) 경로에서 ESCAPE_SEP 이상 떨어진 자리로 먼저 빠진다 (벽 여유 0.17 m 이상).
               빠져나온 뒤에는 우선권 로봇의 남은 경로를 피하는 새 경로를 계속 찾고, 겹치지 않으면 바로 그 길로 출발한다.
               (우선권 로봇이 이미 지나온 길은 점유로 보지 않는다)
  6) 해제: 우선권 로봇의 남은 경로가 양보 로봇의 위치·경로에서 R_CONF 이상 멀어지면 끝. 이후 둘 다 평소대로 주행

decide(agents, now) -> {로봇: Command}  (traffic.TrafficManager.decide 와 같은 모양이라 시뮬레이터·ROS 노드에 그대로 꽂힌다)
  GO            : 평소대로 (path 가 있으면 그 경로를 고정해서 따라간다. hold_index = 경로 번호, 바뀔 때만 다시 보낸다)
  HOLD          : 제자리 정지
  YIELD(path)   : path 끝 자리로 비켜선다
  BACKUP(path)  : 비켜설 자리가 없어 지나온 쪽으로 물러난다
role[로봇] (LCD 용): pause | win | lose | lose_go(양보 후 새 길로 출발)
"""
import math
from dataclasses import dataclass, field

import numpy as np

from .planner import astar
from .traffic import Command, TrafficManager

D_MEET = 0.6          # 마주침 판단 거리 (중심 간, m). 반응 지연(약 0.5 s × 접근 속도 0.4 m/s)을 빼면 약 0.3 m 에서 멈춘다
L_AHEAD = 0.7         # 침범을 볼 내 남은 경로 앞쪽 길이 (m)
R_TUBE = 0.17         # 경로 점유 띠 반폭 (m). 상대 몸체 중심이 이 안이면 침범 (몸체 접촉 한계 0.155 + 여유)
PAUSE = 1.5           # 마주치면 둘 다 멈춰 있는 시간 (s)
ESCAPE_SEP = 0.45     # 양보 로봇이 비켜설 자리와 우선권 로봇 경로(중심선) 사이 최소 거리 (m)
R_TUBE_PARKED = 0.25  # 서 있는 로봇(비켜 주지 않음) 주변은 띠를 넓게 본다 (시뮬: 목표에 선 로봇 옆 0.185 m 로 스치다 충돌)
ESCAPE_RELAX = (0.34, 0.30)   # ESCAPE_SEP 자리를 못 찾으면 차례로 낮춰 다시 찾는다 (막힌 칸 입구 등)
R_CONF = 0.34         # 해제 판단: 우선권 로봇의 남은 경로가 이보다 멀어지면 끝 (= 두 점유 띠가 닿지 않음)
DETOUR_PENALTY = 30.0 # 새 경로 계획 때 우선권 로봇 남은 경로 근처(ESCAPE_SEP 안) 칸에 더하는 비용 (거의 막힌 것처럼)
TIMEOUT = 90.0        # 이 시간 안에 해제되지 않으면 포기하고 평소 주행으로 (s)


def _min_dist(p, path):
    if path is None or len(path) == 0:
        return 9.0
    return float(np.min(np.hypot(path[:, 0] - p[0], path[:, 1] - p[1])))


def _path_gap(a, b):
    """두 경로(점 열) 사이 최소 거리"""
    if a is None or b is None or len(a) == 0 or len(b) == 0:
        return 9.0
    a, b = a[::2], b[::2]
    return float(np.min(np.hypot(a[:, None, 0] - b[None, :, 0], a[:, None, 1] - b[None, :, 1])))


def _trim(path, pos):
    """경로에서 지금 위치 이후만"""
    if path is None or len(path) < 2:
        return path
    i = int(np.argmin(np.hypot(path[:, 0] - pos[0], path[:, 1] - pos[1])))
    return np.vstack([pos, path[i + 1:]]) if i + 1 < len(path) else pos[None, :].copy()


def _length(path):
    return 0.0 if path is None or len(path) < 2 else float(np.sum(np.hypot(*np.diff(path, axis=0).T)))


@dataclass
class Encounter:
    a: str
    b: str
    t0: float
    phase: str = 'PAUSE'              # PAUSE | RESOLVE
    winner: str = None
    loser: str = None
    route_w: np.ndarray = None        # 고정한 우선권 로봇 경로
    route_id: int = 0
    lmode: str = ''                   # 양보 로봇: ESCAPE | BACKUP | AVOID(빠져나옴, 새 길 찾는 중) | GO(새 길로 주행) | STUCK
    esc: np.ndarray = None            # 비켜설 길 (ESCAPE / BACKUP)
    alt: np.ndarray = None            # 양보 로봇의 새 길
    alt_id: int = 0
    alt_t: float = -1e9
    swapped: bool = False
    lgoal: np.ndarray = None          # 양보 로봇의 원래 목표 (비키는 동안 경로가 비킬 자리까지로 바뀌므로 정할 때 기억)
    log: list = field(default_factory=list)


class EncounterManager:
    wants_time = True

    def __init__(self, gm, d_meet=D_MEET, l_ahead=L_AHEAD, r_tube=R_TUBE, pause=PAUSE, escape_sep=ESCAPE_SEP, r_conf=R_CONF):
        self.gm = gm
        self.d_meet, self.l_ahead, self.r_tube, self.pause = d_meet, l_ahead, r_tube, pause
        self.escape_sep, self.r_conf = escape_sep, r_conf
        # 비켜설 자리 찾기·후진 경로는 기존 TrafficManager 의 것을 쓴다 (벽 여유 0.17 m, 1차선 입구 피하기, 상대 현재 위치 근처로 안 지나감)
        self.tm = TrafficManager(gm, r_conf=r_conf, r_conf_parked=min(0.30, r_conf), yield_sep=escape_sep)
        self.enc = {}
        self.prev_d = {}
        self.role = {}
        self.events = []                  # (로봇, 내용) — 노드가 로그로 남긴다
        self._ids = 0
        self.winner = {}                  # 호환용 (fleet_traffic 이 tm.winner 를 볼 때)

    def reset(self):
        self.enc.clear()
        self.prev_d.clear()
        self.role.clear()
        self.events.clear()
        self.tm.reset()

    # ---------- 판단 ----------
    def decide(self, agents, now=0.0):
        by = {a.id: a for a in agents}
        cmds = {a.id: Command('GO') for a in agents}
        self.role = {}
        for key, e in list(self.enc.items()):
            if e.a not in by or e.b not in by or now - e.t0 > TIMEOUT:
                if e.a in by and e.b in by:
                    self._event(e.a, f'{e.b} 와의 마주침을 {TIMEOUT:.0f}초 안에 풀지 못해 평소 주행으로 돌아갑니다')
                del self.enc[key]
                continue
            if not self._step(e, by, agents, now, cmds):
                del self.enc[key]
        busy = {x for e in self.enc.values() for x in (e.a, e.b)}
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                a, b = agents[i], agents[j]
                key = frozenset((a.id, b.id))
                d = float(np.hypot(*(a.pos - b.pos)))
                prev = self.prev_d.get(key)
                self.prev_d[key] = d
                if a.id in busy or b.id in busy or key in self.enc or (a.parked and b.parked):
                    continue
                approaching = prev is None or d < prev - 0.002     # 처음 보는 쌍(이미 붙어 있음)은 가까워지는 중으로 본다
                if d > self.d_meet or not (approaching or d < 0.35) or not self._los(a.pos, b.pos):
                    continue                          # 멀거나, 가까워지는 중이 아니거나(아주 가까우면 상관없이), 사이에 벽
                ab, ba = self._intrudes(a, b), self._intrudes(b, a)
                if not (ab or ba):
                    continue                          # 서로의 점유 띠 밖: 스쳐 지나가는 중
                e = Encounter(a.id, b.id, now)
                self.enc[key] = e
                busy |= {a.id, b.id}
                self._event(a.id, f'{b.id} 와 마주침 ({d:.2f} m): 둘 다 {self.pause:.1f}초 정지')
                cmds[a.id] = Command('HOLD', 0, why=f'{b.id} 와 마주침: 정지')
                cmds[b.id] = Command('HOLD', 0, why=f'{a.id} 와 마주침: 정지')
                self.role[a.id] = self.role[b.id] = 'pause'
        self.winner = {(e.a, e.b): e.winner for e in self.enc.values() if e.winner}
        return cmds

    def _step(self, e, by, agents, now, cmds):
        """진행 중인 마주침 한 건. False 를 돌려주면 해제"""
        a, b = by[e.a], by[e.b]
        if e.phase == 'PAUSE':
            for x, o in ((a, b), (b, a)):
                if not x.parked:
                    cmds[x.id] = Command('HOLD', 0, why=f'{o.id} 와 마주침: 정지')
                self.role[x.id] = 'pause'
            if now - e.t0 < self.pause:
                return True
            self._resolve(e, a, b, agents)
            e.phase = 'RESOLVE'
        W, L = by[e.winner], by[e.loser]
        rem_w = _trim(e.route_w, W.pos)
        # ----- 해제: 우선권 로봇의 남은 경로가 양보 로봇의 위치와 (새) 경로에서 충분히 멀어짐 -----
        lpath = e.alt if e.lmode == 'GO' and e.alt is not None else (None if L.parked else L.plan)
        far_pos = _min_dist(L.pos, rem_w) >= self.r_conf
        far_path = lpath is None or len(lpath) < 2 or _path_gap(_trim(lpath, L.pos), rem_w) >= self.r_conf
        if (len(rem_w) < 3 or W.parked) and _min_dist(L.pos, W.pos[None, :]) > self.r_conf:
            far_path = far_pos = True                 # 우선권 로봇이 도착했다
        if far_pos and far_path and e.lmode in ('AVOID', 'GO', 'STUCK'):
            self._event(e.loser, f'{e.winner} 가 지나감: 마주침 해제')
            return False
        # ----- 우선권 로봇: 양보 로봇 몸체가 내 남은 경로 띠에서 빠지면 고정 경로로 출발 -----
        if _min_dist(L.pos, rem_w[: int(self.l_ahead * 2 / W.ds) + 1]) < self.r_tube or e.lmode == 'STUCK':
            cmds[W.id] = Command('HOLD', 0, why=f'{L.id} 가 비키는 중 대기')
        else:
            cmds[W.id] = Command('GO', e.route_id, path=rem_w, why=f'{L.id} 와 마주침: 우선 주행')
        self.role[W.id] = 'win'
        # ----- 양보 로봇 -----
        self.role[L.id] = 'lose'
        if e.lmode in ('ESCAPE', 'BACKUP'):
            done = _min_dist(L.pos, e.esc[-1:]) < 0.08 or _min_dist(L.pos, rem_w) >= self.escape_sep
            if not done:
                cmds[L.id] = Command('YIELD' if e.lmode == 'ESCAPE' else 'BACKUP', path=_trim(e.esc, L.pos),
                                     why=f'{W.id} 경로에서 비켜서기' if e.lmode == 'ESCAPE' else f'{W.id} 와 마주침: 뒤로 빠져 길 비키기')
                return True
            e.lmode = 'AVOID'
            self._event(L.id, f'{W.id} 경로에서 빠져나옴 ({_min_dist(L.pos, rem_w):.2f} m): 새 길 찾는 중')
        if e.lmode == 'STUCK':
            cmds[L.id] = Command('HOLD', 0, why=f'{W.id} 와 마주쳤지만 둘 다 비킬 곳 없음')
            return True
        if L.parked:                                  # 미션 없는 로봇: 비켜선 자리에서 기다린다
            cmds[L.id] = Command('HOLD', 0, why=f'{W.id} 통과 대기')
            return True
        if e.lmode == 'AVOID' and now - e.alt_t > 1.0:
            e.alt_t = now
            self._goal_for = {L.id: e.lgoal}
            alt = self._detour(L, rem_w)
            if alt is not None and _path_gap(alt, rem_w) >= self.r_conf:
                e.alt, e.lmode = alt, 'GO'
                self._ids += 1
                e.alt_id = self._ids
                self._event(L.id, f'{W.id} 경로와 겹치지 않는 새 길({_length(alt):.2f} m)로 출발')
        if e.lmode == 'GO':
            cmds[L.id] = Command('GO', e.alt_id, path=_trim(e.alt, L.pos), why=f'{W.id} 를 피하는 새 길로 주행')
            self.role[L.id] = 'lose_go'
        else:
            cmds[L.id] = Command('HOLD', 0, why=f'{W.id} 통과 대기')
        return True

    def _resolve(self, e, a, b, agents):
        """우선권을 정하고 우선권 로봇 경로를 고정, 양보 로봇의 비켜설 길을 정한다"""
        if a.parked != b.parked:
            loser = a if a.parked else b
            winner = b if a.parked else a
        else:
            ca, cb = self._escape(a, b, agents), self._escape(b, a, agents)
            la, lb = (_length(ca[1]) if ca else 1e9), (_length(cb[1]) if cb else 1e9)
            if abs(la - lb) < 0.10:                   # 비키는 수고가 비슷하면 목표에 가까운 쪽이 우선
                ra, rb = _length(a.plan), _length(b.plan)
                winner, loser = (a, b) if (ra < rb or (ra == rb and a.id < b.id)) else (b, a)
            else:
                winner, loser = (b, a) if la < lb else (a, b)
        self._assign(e, winner, loser, agents)

    def _assign(self, e, winner, loser, agents):
        e.winner, e.loser = winner.id, loser.id
        e.lgoal = None if loser.parked or len(loser.plan) < 2 else loser.plan[-1].copy()
        e.route_w = winner.plan.copy() if len(winner.plan) > 1 else winner.pos[None, :].copy()
        self._ids += 1
        e.route_id = self._ids
        esc = self._escape(loser, winner, agents)
        if esc is not None:
            e.lmode, e.esc = esc
            self._event(loser.id, f'{winner.id} 에게 양보: ' + ('비켜설 자리' if esc[0] == 'ESCAPE' else '뒤로 물러날 자리')
                        + f' ({esc[1][-1][0]:.2f}, {esc[1][-1][1]:.2f}) 로 이동 ({_length(esc[1]):.2f} m)')
            return
        if not e.swapped:                             # 양보할 곳이 없으면 역할을 바꿔 본다
            e.swapped = True
            esc2 = self._escape(winner, loser, agents)
            if esc2 is not None:
                self._event(loser.id, f'비킬 곳이 없어 {winner.id} 가 대신 양보')
                return self._assign(e, loser, winner, agents)
        e.lmode, e.esc = 'STUCK', None
        self._event(loser.id, f'{winner.id} 와 마주쳤지만 둘 다 비킬 곳이 없습니다 (사람 확인 필요)')

    def _escape(self, lose, win, agents):
        """lose 가 win 의 남은 경로에서 ESCAPE_SEP 이상 떨어진 자리로 가는 길: ('ESCAPE', path) / ('BACKUP', path) / None"""
        keep = (self.tm.r_conf_parked, self.tm.yield_sep)
        try:
            for sep in (self.escape_sep,) + tuple(s for s in ESCAPE_RELAX if s < self.escape_sep):
                self.tm.r_conf_parked = self.tm.yield_sep = sep      # 미션 없이 서 있던 로봇도 같은 거리만큼 비킨다
                c = self.tm._yield(lose, win, agents, static=True)   # win 은 제자리에서 기다린다고 보고 그 옆으로 지나가지 않는다
                if c.kind == 'YIELD' and c.path is not None and len(c.path) > 1:
                    if sep < self.escape_sep:
                        self._event(lose.id, f'{self.escape_sep:.2f} m 떨어진 자리가 없어 {sep:.2f} m 기준으로 비킵니다')
                    return ('ESCAPE', c.path)
        finally:
            self.tm.r_conf_parked, self.tm.yield_sep = keep
        return None

    def _detour(self, L, rem_w):
        """양보 로봇의 새 길: 우선권 로봇 남은 경로 근처(ESCAPE_SEP 안)를 매우 비싸게 보고 계획"""
        goal = getattr(self, '_goal_for', {}).get(L.id)
        if goal is None:
            return None
        near = self.gm.raster(rem_w) < self.escape_sep
        return astar(self.gm, L.pos, goal, extra_cost=np.where(near, DETOUR_PENALTY, 0.0))

    # ---------- 보조 ----------
    def _intrudes(self, me, other):
        """other 몸체 중심이 내 남은 경로 앞 L_AHEAD 의 점유 띠(±R_TUBE) 안인가"""
        if me.parked or len(me.plan) < 2:
            return False
        ahead = me.plan[: int(self.l_ahead / me.ds) + 1]
        return _min_dist(other.pos, ahead) < (max(self.r_tube, R_TUBE_PARKED) if other.parked else self.r_tube)

    def _los(self, p, q):
        d = float(np.hypot(*(q - p)))
        n = max(2, int(d / 0.02))
        pts = p + (q - p) * np.linspace(0.0, 1.0, n)[:, None]
        return bool(np.all(self.gm.clearance_at(pts) > 0.0))

    def _event(self, rid, msg):
        self.events.append((rid, msg))
