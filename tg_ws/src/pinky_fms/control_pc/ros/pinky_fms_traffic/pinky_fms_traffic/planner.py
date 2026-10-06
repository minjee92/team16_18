"""단일 로봇 경로 (A*). Nav2 NavFn 처럼 벽에 가까울수록 비용을 더해 통로 가운데로 가게 한다."""
import heapq
import math

import numpy as np

R_PLAN = 0.07          # 이보다 벽에 가까운 칸은 지나가지 않는다 (m)
NEI = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def passable(gm, r_plan=R_PLAN):
    return gm.free & (gm.clear >= r_plan)


def snap(gm, mask, p):
    """p 가 지나갈 수 없는 칸이면 가장 가까운 지나갈 수 있는 칸으로"""
    r, c = gm.cell(*p)
    if gm.inside(r, c) and mask[r, c]:
        return r, c
    rr, cc = np.nonzero(mask)
    k = np.argmin((rr - r) ** 2 + (cc - c) ** 2)
    return int(rr[k]), int(cc[k])


def astar(gm, start, goal, mask=None, extra_cost=None, wall_k=0.6):
    mask = passable(gm) if mask is None else mask
    s, g = snap(gm, mask, start), snap(gm, mask, goal)
    cost_cell = wall_k * np.exp(-10.0 * np.clip(gm.clear - R_PLAN, 0, None))
    if extra_cost is not None:
        cost_cell = cost_cell + extra_cost
    openq = [(0.0, s)]
    gs = {s: 0.0}
    came = {}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == g:
            break
        for dr, dc in NEI:
            n = (cur[0] + dr, cur[1] + dc)
            if not gm.inside(*n) or not mask[n]:
                continue
            step = math.hypot(dr, dc) * gm.res
            ng = gs[cur] + step * (1.0 + cost_cell[n])
            if ng < gs.get(n, 1e18):
                gs[n] = ng
                came[n] = cur
                h = math.hypot(n[0] - g[0], n[1] - g[1]) * gm.res
                heapq.heappush(openq, (ng + h, n))
    if g not in came and g != s:
        return None
    cells = [g]
    while cells[-1] != s:
        cells.append(came[cells[-1]])
    pts = np.array([gm.world(*c) for c in cells[::-1]])
    pts[0] = start if mask[gm.cell(*start)] else pts[0]
    return resample(pts, gm.res)


def resample(pts, ds):
    if len(pts) < 2:
        return np.array(pts)
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(2, int(s[-1] / ds) + 1)
    t = np.linspace(0, s[-1], n)
    return np.stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])], axis=1)
