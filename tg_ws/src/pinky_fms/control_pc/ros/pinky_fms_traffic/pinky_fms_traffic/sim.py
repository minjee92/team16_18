"""2D 질점 시뮬레이터: 로봇이 경로를 따라 0.2 m/s 로 움직이고, 충돌(중심 거리 < 2*R_BODY)·교착을 센다."""
import math
import random

import numpy as np

from .planner import astar
from .traffic import Agent, Command, R_BODY, TrafficManager

DS = 0.02


class SimRobot:
    def __init__(self, rid, start, goal, speed=0.2, delay=0.0, fixed=False):
        self.id, self.pos, self.goal = rid, np.array(start, float), np.array(goal, float)
        self.fixed = fixed            # 관제 명령을 따르지 않는 로봇 (차선 주행 흉내): 앞에 다른 로봇이 바짝 있을 때만 선다
        self.speed, self.delay = speed, delay
        self.path = None
        self.i = 0.0
        self.hold = None              # 이 인덱스까지만 진행
        self.mode = 'FOLLOW'          # FOLLOW | YIELD | WAIT | PARKED
        self.done_t = None
        self.dist = 0.0
        self.waited = 0.0
        self.trail = [np.array(start, float)]   # 지나온 길
        self.yaw = 0.0
        self.back_left = 0.0                      # BACKUP_STRAIGHT: 남은 후진 거리
        self.finished = False         # 목표에 도착한 적 있음 (비켜섰다가 목표로 되돌아가지 않는다)


def _front_blocked(r, robots, ahead=0.30, half=0.16):
    """r 의 앞(진행 방향) ahead m, 좌우 half m 안에 다른 로봇 중심이 있는가"""
    c, s = math.cos(r.yaw), math.sin(r.yaw)
    for o in robots:
        if o is r:
            continue
        dx, dy = o.pos[0] - r.pos[0], o.pos[1] - r.pos[1]
        fx, fy = dx * c + dy * s, -dx * s + dy * c
        if 0.0 < fx <= ahead and abs(fy) <= half:
            return True
    return False


def run(gm, tm, specs, T=200.0, dt=0.1, tick=0.5, use_traffic=True, log=None, pos_noise=0.0, seed=0):
    tm.reset()          # 이전 시험의 우선권 기록이 남지 않게
    robots = [SimRobot(*s) if not isinstance(s, SimRobot) else s for s in specs]
    for r in robots:
        r.path = astar(gm, r.pos, r.goal)
        if r.path is None:
            return {'error': 'no path', 'robot': r.id}
    t, min_sep, coll, last_move, nexttick = 0.0, 9.0, 0, 0.0, 0.0
    rngn = random.Random(seed)
    bias = {r.id: np.zeros(2) for r in robots}      # 위치추정(AMCL) 오차: 서서히 변하는 편향
    first_coll = None
    while t < T:
        if t >= nexttick:
            nexttick += tick
            if use_traffic:
                agents = []
                for r in robots:
                    if pos_noise > 0:
                        bias[r.id] = np.clip(bias[r.id] * 0.95 + np.array([rngn.gauss(0, pos_noise * 0.3), rngn.gauss(0, pos_noise * 0.3)]), -pos_noise * 1.5, pos_noise * 1.5)
                    sh = bias[r.id]
                    if r.mode == 'PARKED':
                        plan, parked = r.pos[None, :] + sh, True
                    else:
                        if r.mode == 'WAIT':
                            p = astar(gm, r.pos, r.goal)
                            r.path, r.i = p, 0.0
                        plan = r.path[int(r.i):] + sh
                        parked = False
                    tr = np.array(r.trail[-int(2.5 / DS):]) + sh if len(r.trail) > 1 else None
                    agents.append(Agent(r.id, r.pos + sh, plan, DS, 0.2, parked, r.mode in ('YIELD', 'BACKUP', 'BSTRAIGHT'), trail=tr, backing=r.mode in ('BACKUP', 'BSTRAIGHT'), yaw=r.yaw, fixed=r.fixed))
                cmds = tm.decide(agents, t) if getattr(tm, 'wants_time', False) else tm.decide(agents)
                for r in robots:
                    c = cmds[r.id]
                    if r.fixed:                                 # 관제 명령을 따르지 않는다
                        continue
                    if r.mode in ('BACKUP', 'BSTRAIGHT'):      # 후진은 끝날 때까지 계속
                        continue
                    if c.kind == 'BACKUP_STRAIGHT':
                        r.prev_mode = 'PARKED' if (r.mode == 'PARKED' or r.finished) else 'WAIT'
                        r.mode, r.back_left, r.hold = 'BSTRAIGHT', c.hold_index / 100.0, None
                        if log: log(t, r.id, c.why)
                        continue
                    if c.kind == 'BACKUP':
                        r.path, r.i, r.mode, r.hold = c.path - bias[r.id], 0.0, 'BACKUP', None
                        if log: log(t, r.id, c.why)
                        continue
                    if r.mode in ('PARKED',) or (r.mode == 'FOLLOW' and len(r.path) <= 1):
                        if c.kind == 'YIELD':
                            r.path, r.i, r.mode, r.hold, r.finished = c.path, 0.0, 'YIELD', None, True
                            if log: log(t, r.id, c.why)
                        continue
                    if r.mode == 'YIELD':
                        r.hold = None
                        if c.kind == 'YIELD' and np.hypot(*(c.path[-1] - r.path[-1])) > 0.05:    # 비켜설 자리가 바뀌었다 (같으면 가던 길 유지)
                            r.path, r.i = c.path, 0.0
                        elif c.kind == 'HOLD':                 # 비켜설 곳이 없어졌다: 멈춘다
                            r.path, r.i = r.pos[None, :].copy(), 0.0
                        continue
                    if c.kind == 'GO':
                        if r.mode == 'WAIT':
                            r.mode = 'FOLLOW'
                        r.hold = None
                        if c.path is not None and len(c.path) > 1 and getattr(r, 'route_id', None) != c.hold_index:
                            r.path, r.i, r.route_id = c.path.copy(), 0.0, c.hold_index      # 정해 준 경로로 (마주침: 고정 경로 / 새 길)
                    elif c.kind == 'HOLD':
                        if r.mode == 'WAIT':
                            r.hold = 0
                        else:
                            r.hold = int(r.i) + c.hold_index
                        if log and (r.hold is not None): log(t, r.id, c.why)
                    elif c.kind == 'YIELD':
                        r.path, r.i, r.mode, r.hold = c.path, 0.0, 'YIELD', None
                        if log: log(t, r.id, c.why)
        moving = False
        for r in robots:
            if r.mode == 'PARKED' or t < r.delay:
                continue
            if r.mode == 'WAIT':
                r.waited += dt
                continue
            if r.mode == 'BSTRAIGHT':                    # 방향 그대로 똑바로 후진
                step = min(r.speed * 0.25 * dt, r.back_left)
                r.pos = r.pos - step * np.array([math.cos(r.yaw), math.sin(r.yaw)])
                r.back_left -= step
                moving = True
                if r.back_left <= 1e-6:
                    r.mode = r.prev_mode
                    if r.mode == 'WAIT':
                        r.path, r.i = r.pos[None, :].copy(), 0.0
                continue
            lim = len(r.path) - 1
            if r.mode == 'FOLLOW' and r.hold is not None:
                lim = min(lim, r.hold)
            if r.fixed and _front_blocked(r, robots):      # 차선 로봇의 앞 사물 정지 흉내 (초음파·라이다 앞 통로)
                lim = int(r.i)
            step = (r.speed * 0.5 if r.mode == 'BACKUP' else r.speed) * dt / DS     # 후진은 천천히
            if r.i + 1e-9 < lim:
                r.i = min(lim, r.i + step)
                new = r.path[int(r.i)]
                if r.mode != 'BACKUP' and np.hypot(*(new - r.pos)) > 1e-6:
                    r.yaw = math.atan2(new[1] - r.pos[1], new[0] - r.pos[0])
                r.dist += float(np.hypot(*(new - r.pos)))
                r.pos = new.copy()
                moving = True
                if r.mode != 'BACKUP' and np.hypot(*(r.pos - r.trail[-1])) >= DS:
                    r.trail.append(r.pos.copy())
                elif r.mode == 'BACKUP':
                    while len(r.trail) > 1 and np.hypot(*(r.trail[-1] - r.pos)) < DS * 2:   # 되돌아간 만큼 지나온 길에서 지운다
                        r.trail.pop()
            else:
                r.waited += dt
                if r.mode == 'FOLLOW' and r.i >= len(r.path) - 1 - 1e-9:
                    r.mode, r.done_t, r.finished = 'PARKED', t, True
                elif r.mode in ('YIELD', 'BACKUP') and r.i >= len(r.path) - 1 - 1e-9:
                    r.mode = 'PARKED' if r.finished else 'WAIT'
        for a in range(len(robots)):
            for b in range(a + 1, len(robots)):
                d = float(np.hypot(*(robots[a].pos - robots[b].pos)))
                min_sep = min(min_sep, d)
                if d < 2 * R_BODY:
                    coll += 1
                    first_coll = first_coll if first_coll is not None else t
        if moving:
            last_move = t
        if all(r.mode == 'PARKED' for r in robots):
            break
        if t - last_move > 30.0:
            return {'result': 'DEADLOCK', 't': t, 'min_sep': min_sep, 'collision_frames': coll, 'first_collision': first_coll,
                    'modes': {r.id: r.mode for r in robots}}
        t += dt
    ok = all(r.mode == 'PARKED' for r in robots)
    return {'result': 'OK' if ok else 'TIMEOUT', 't': round(t, 1), 'min_sep': round(min_sep, 3), 'collision_frames': coll,
            'first_collision': first_coll, 'waited': {r.id: round(r.waited, 1) for r in robots}, 'dist': {r.id: round(r.dist, 2) for r in robots}}
