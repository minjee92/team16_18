"""차선 주행(Lane Following) 로봇끼리의 마주침 판단과 비켜설 자리 계산 (ROS 없음, 단위 시험 가능).

지도 두 장을 쓴다 (GUI 지도 목록의 두 장과 같다):
  lanes_yaml : 차선만 빈 칸인 지도 (mission4_3_lanes_1cm) → 차선 띠
  floor_yaml : 차선을 지운 지도 (mission4_3_nolanes_1cm) → 바닥과 벽
차선 폭(약 0.21 m)이 로봇 두 대(0.12 m×2)보다 좁아 차선 안에서는 비킬 수 없다. 그래서 양보 로봇은 차선 밖 바닥의
'비켜설 자리'로 잠깐 나갔다가, 떠난 자리·방향으로 돌아와 차선 추종을 재개한다.
비켜설 자리 조건: 몸체가 차선 띠에서 벗어나고 차선 중심선에서 center_clear 이상 떨어져
우선권 로봇의 앞 사물 감시 통로(±0.10 m) 밖에 있고, 벽에서 떨어져 있으며, 지금 위치에서 직선으로 갈 수 있고
(제자리 회전 → 직진), 그 직선이 상대 로봇 옆을 지나지 않는다.
"""
import math

import numpy as np
from scipy import ndimage as ndi

from .gridmap import GridMap

BODY_HALF = 0.06          # 로봇 몸체 반폭 (m)


def ang_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def seg_point_dist(a, b, p):
    """선분 a-b 와 점 p 사이 거리 (a, b 는 (2,) 또는 (N,2))"""
    a, b, p = np.atleast_2d(a), np.atleast_2d(b), np.asarray(p, dtype=float)
    ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0.0, 1.0)
    return np.hypot(*(a + ab * t[:, None] - p).T)


class LaneGeometry:
    def __init__(self, lanes_yaml, floor_yaml, res=0.02, center_clear=0.19, wall_margin=0.03, center_pref=0.23, pass_clear=0.065):
        # center_clear: 차선 중심선 ~ 비켜설 자리(로봇 중심) 최소 거리. 우선권 로봇은 차선 중심을 따라가며 앞 ±0.10 m 통로의
        #   라이다 점을 사물로 보고 멈춘다 → 0.10(통로) + 0.06(몸체 반폭) + 여유 0.03.
        #   0.21(여유 0.05)이면 벽·상자 옆 구간(중심~벽 약 0.2 m)에서 49% 가 옆자리가 없어 0.19 로 둔다 (0.19 이면 0%).
        #   여유가 빠듯하므로 center_pref 보다 가까운 자리는 비용을 더해, 같은 거리면 중심에서 먼 자리를 고른다.
        #   차선 띠 폭이 곳마다 달라(반폭 0.075~0.105 m) 띠 가장자리 기준으로 재면 좁은 곳에서 통로에 걸린다 (격리 시험 headon_wall).
        self.lane = GridMap(lanes_yaml, res)
        self.floor = GridMap(floor_yaml, res)
        if (self.lane.H, self.lane.W) != (self.floor.H, self.floor.W):
            raise ValueError('차선 지도와 바닥 지도의 크기가 다릅니다')
        self.res = self.floor.res
        band = self.lane.free
        self.lane_dist = ndi.distance_transform_edt(~band) * self.res      # 차선 띠까지 거리 (띠 안 = 0)
        # 차선 중심선: 띠 안에서 가장자리까지 거리가 주변(5×5) 최대인 칸
        clr = np.where(band, ndi.distance_transform_edt(band), 0.0)
        center = band & (clr >= ndi.maximum_filter(clr, size=5) - 0.5)
        self.center = center
        self.band = band
        self.center_clear, self.center_pref = center_clear, center_pref
        self.pass_clear = pass_clear                                       # 비켜서는 직선 위 모든 점의 최소 벽 여유
        esc = (self.floor.free & (self.floor.clear >= BODY_HALF + wall_margin)
               & (self.lane_dist >= BODY_HALF + 0.02))
        rr, cc = np.nonzero(esc)
        self.esc_xy = np.stack([self.floor.ox + (cc + 0.5) * self.res, self.floor.oy + (rr + 0.5) * self.res], axis=1)

    # ---------- 지도 조회 ----------
    def _rc(self, pts):
        pts = np.atleast_2d(pts)
        r = np.clip(((pts[:, 1] - self.floor.oy) / self.res).astype(int), 0, self.floor.H - 1)
        c = np.clip(((pts[:, 0] - self.floor.ox) / self.res).astype(int), 0, self.floor.W - 1)
        return r, c

    def lane_dist_at(self, p):
        r, c = self._rc(p)
        return float(self.lane_dist[r[0], c[0]])

    def on_lane(self, p, tol=0.05):
        return self.lane_dist_at(p) <= tol

    def wall_between(self, a, b):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res * 0.5)) + 1)
        r, c = self._rc(np.stack([np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n)], axis=1))
        return bool(self.floor.occ[r, c].any())

    def local_centerline(self, p, reach=0.6):
        """p 에서 차선 띠를 따라 reach 안에 있는 차선 중심선 점 (마주친 구간만. 옆을 지나는 다른 차선 구간은 빼야
        그 사이 바닥을 비켜설 자리로 쓸 수 있다)"""
        r, c = self._rc(p)
        seed = np.zeros_like(self.band)
        rr, cc = np.nonzero(self.band[max(r[0] - 3, 0):r[0] + 4, max(c[0] - 3, 0):c[0] + 4])
        seed[rr + max(r[0] - 3, 0), cc + max(c[0] - 3, 0)] = True       # 위치 오차로 띠 밖이어도 가까운 띠 칸에서 시작
        reached = ndi.binary_dilation(seed, structure=np.ones((3, 3), bool), iterations=int(reach / self.res), mask=self.band)
        rr, cc = np.nonzero(reached & self.center)
        return np.stack([self.floor.ox + (cc + 0.5) * self.res, self.floor.oy + (rr + 0.5) * self.res], axis=1)

    def _geo_levels(self, p, reach):
        """p 에서 차선 띠를 따라간 거리(칸 수, 8방향) — 닿지 않은 칸은 -1"""
        r, c = self._rc(p)
        lv = np.full(self.band.shape, -1, int)
        cur = np.zeros_like(self.band)
        r0, c0 = max(r[0] - 3, 0), max(c[0] - 3, 0)
        rr, cc = np.nonzero(self.band[r0:r[0] + 4, c0:c[0] + 4])
        cur[rr + r0, cc + c0] = True
        lv[cur] = 0
        st = np.ones((3, 3), bool)
        for k in range(1, int(reach / self.res) + 1):
            nxt = ndi.binary_dilation(cur, structure=st, mask=self.band) & (lv < 0)
            if not nxt.any():
                break
            lv[nxt] = k
            cur = cur | nxt
        return lv

    def priority_path(self, prio_xy, toward_xy, reach=1.0):
        """우선권 로봇(prio_xy)이 toward_xy(양보 로봇이 있던 곳) 쪽으로 차선을 따라 지나갈 중심선 점과 진행 방향.
        반환 (pts (N,2), tangents (N,2)). 우선권 로봇 뒤쪽 구간은 뺀다."""
        lp, ly = self._geo_levels(prio_xy, reach), self._geo_levels(toward_xy, reach + 0.6)
        D = ly[self._rc(prio_xy)[0][0], self._rc(prio_xy)[1][0]]
        D = D if D >= 0 else int(math.hypot(toward_xy[0] - prio_xy[0], toward_xy[1] - prio_xy[1]) / self.res)
        sel = self.center & (lp >= 0) & (ly >= 0) & (ly < lp + D - 3)      # 뒤쪽 구간: ly ≈ lp + D
        rr, cc = np.nonzero(sel)
        if not len(rr):
            return np.zeros((0, 2)), np.zeros((0, 2))
        pts = np.stack([self.floor.ox + (cc + 0.5) * self.res, self.floor.oy + (rr + 0.5) * self.res], axis=1)
        g = lp[rr, cc]
        tan = np.zeros_like(pts)
        for i in range(len(pts)):
            m = (g > g[i] + 1) & (g <= g[i] + 5) & (np.hypot(*(pts - pts[i]).T) < 0.14)
            if m.any():
                v = pts[m].mean(0) - pts[i]
            else:                                            # 끝점: 앞선 점들에서 이어 온 방향
                m = (g < g[i]) & (g >= g[i] - 4) & (np.hypot(*(pts - pts[i]).T) < 0.14)
                v = pts[i] - pts[m].mean(0) if m.any() else np.zeros(2)
            n = np.hypot(*v)
            tan[i] = v / n if n > 1e-6 else 0.0
        return pts, tan

    @staticmethod
    def in_corridor(xy, path, ahead=0.30, half=0.18):
        """xy(로봇 중심, (K,2))가 우선권 로봇의 앞 사물 감시 상자에 걸리는가. 상자: 경로 각 점에서 그 점의 진행 방향으로
        앞 ahead(앞면 0.07 + 정지 거리 0.15 + 상대 몸체 0.06 ≈ 0.28), 좌우 half(통로 0.10 + 몸체 0.06 + 0.02).
        곡선에서는 로봇 정면이 차선 밖 안쪽을 향하므로 중심선 거리만으로는 부족하다 (격리 시험 headon_curve)."""
        pts, tan = path
        xy = np.atleast_2d(xy)
        if not len(pts):
            return np.zeros(len(xy), bool)
        rel = xy[:, None, :] - pts[None, :, :]                       # (K, N, 2)
        fx = (rel * tan[None]).sum(-1)
        fy = rel[..., 0] * tan[None, :, 1] - rel[..., 1] * tan[None, :, 0]
        valid = np.hypot(tan[:, 0], tan[:, 1])[None] > 0.5
        return ((fx >= -BODY_HALF) & (fx <= ahead) & (np.abs(fy) <= half) & valid).any(1)

    def find_junctions(self, ring=(0.14, 0.20), min_gap_deg=40.0, merge=0.2):
        """차선 갈림길(세 갈래 이상) 중심 목록. 중심선 점마다 반경 ring 고리 위 중심선 점들의 방향을 묶어 갈래 수를 센다."""
        rr, cc = np.nonzero(self.center)
        pts = np.stack([self.floor.ox + (cc + 0.5) * self.res, self.floor.oy + (rr + 0.5) * self.res], axis=1)
        hits = []
        for p in pts:
            d = np.hypot(*(pts - p).T)
            rg = pts[(d >= ring[0]) & (d <= ring[1])]
            if len(rg) < 3:
                continue
            a = np.sort(np.arctan2(rg[:, 1] - p[1], rg[:, 0] - p[0]))
            gaps = np.diff(np.concatenate([a, [a[0] + 2 * np.pi]]))
            if (gaps > math.radians(min_gap_deg)).sum() >= 3:
                hits.append(p)
        groups = []
        for p in hits:
            for gr in groups:
                if np.hypot(*(np.mean(gr, 0) - p)) < merge:
                    gr.append(p)
                    break
            else:
                groups.append([p])
        return [tuple(float(v) for v in np.mean(gr, 0)) for gr in groups]

    # ---------- 비켜설 자리 ----------
    def escape_for(self, pose, other_xy, max_dist=0.45, other_clear=0.22, turn_w=0.08, other_pose=None, center_clear=None):
        """pose(x, y, yaw) 로봇이 other_xy 로봇을 피해 비켜설 자리. 반환 (비용, (x, y)) 또는 None.
        비용 = 직선거리 + turn_w × 회전각(rad) — 비용이 작은 쪽이 '비켜서기 쉬운 쪽'."""
        p = np.array(pose[:2], dtype=float)
        o = np.array(other_xy, dtype=float)
        d = np.hypot(*(self.esc_xy - p).T)
        cand = self.esc_xy[(d <= max_dist) & (np.hypot(*(self.esc_xy - o).T) >= other_clear + BODY_HALF)]
        if not len(cand):
            return None
        # 마주친 구간의 차선 중심선(우선권 로봇이 지나갈 곳)에서 center_clear 이상
        cl = np.concatenate([self.local_centerline(pose), self.local_centerline(other_xy)])
        dc = (np.sqrt(((cand[:, None, :] - cl[None, :, :]) ** 2).sum(-1)).min(1) if len(cl)
              else np.full(len(cand), self.center_pref))
        keep = dc >= (self.center_clear if center_clear is None else center_clear)
        cand, dc = cand[keep], dc[keep]
        if not len(cand):
            return None
        # 우선권 로봇(상대)이 이 로봇 쪽으로 지나갈 때 정면 감시 상자에 걸리면 안 된다 (곡선 구간)
        keep = ~self.in_corridor(cand, self.priority_path(o, p))
        if other_pose is not None:      # 다시 비켜서기: 상대의 실제 위치·방향 기준 정면 상자 (곡선에서 안쪽으로 깎아 도는 만큼)
            h = np.array([[math.cos(other_pose[2]), math.sin(other_pose[2])]])
            keep &= ~self.in_corridor(cand, (o[None], h), ahead=0.45, half=0.20)
        cand, dc = cand[keep], dc[keep]
        if not len(cand):
            return None
        # 직선 경로가 상대 옆을 지나면 안 된다. 이미 그보다 가까이 붙어 있으면(실물 10-09 00:12, 0.19 m) 출발점부터
        # 조건을 못 지키므로 '지금보다 더 가까워지지 않는' 길이면 된다
        keep = seg_point_dist(np.repeat(p[None], len(cand), 0), cand, o) >= min(other_clear, float(np.hypot(*(p - o))) - 0.01)
        cand, dc = cand[keep], dc[keep]
        if not len(cand):
            return None
        # 직선 경로 위의 벽 여유 (출발점 근처 BODY_HALF 는 로봇이 지금 서 있는 자리라 검사하지 않는다)
        n = 24
        t = np.linspace(0.0, 1.0, n)
        pts = p[None, None] + (cand - p)[:, None] * t[None, :, None]          # (K, n, 2)
        seg_len = np.hypot(*(cand - p).T)
        r, c = self._rc(pts.reshape(-1, 2))
        clear = self.floor.clear[r, c].reshape(len(cand), n)
        near_start = (t[None] * seg_len[:, None]) < BODY_HALF
        ok = ((clear >= self.pass_clear) | near_start).all(1)
        cand, seg_len, dc = cand[ok], seg_len[ok], dc[ok]
        if not len(cand):
            return None
        bearing = np.arctan2(cand[:, 1] - p[1], cand[:, 0] - p[0])
        turn = np.abs((bearing - pose[2] + np.pi) % (2 * np.pi) - np.pi)
        cost = seg_len + turn_w * turn + 2.0 * np.clip(self.center_pref - dc, 0.0, None)   # 중심에서 1 cm 덜 떨어지면 2 cm 더 먼 것과 같게
        k = int(np.argmin(cost))
        return float(cost[k]), (float(cand[k, 0]), float(cand[k, 1]))


def retreat_then_escape(ga, pose, other_xy, reach=0.6, max_straight=0.5, other_clear=0.30):
    """바로 비켜설 자리가 없을 때: 차선을 따라 뒤로(진행 반대) 물러날 차선 위 점 q 와, q 에서 비켜설 자리.
    q 는 지금 위치에서 직선으로 갈 수 있고(max_straight 안, 벽 여유) 상대에서 other_clear 이상. 반환 ((qx,qy), escape_xy) 또는 None."""
    p = np.asarray(pose[:2], dtype=float)
    o = np.asarray(other_xy, dtype=float)
    h = np.array([math.cos(pose[2]), math.sin(pose[2])])
    cl = ga.local_centerline(pose, reach)
    if not len(cl):
        return None
    d = np.hypot(*(cl - p).T)
    ok = ((cl - p) @ h < -0.08) & (d <= max_straight) & (np.hypot(*(cl - o).T) >= other_clear)
    for k in np.argsort(np.where(ok, d, np.inf)):
        if not ok[k]:
            break
        q = cl[k]
        n = max(2, int(d[k] / (ga.res * 0.5)) + 1)
        pts = np.stack([np.linspace(p[0], q[0], n), np.linspace(p[1], q[1], n)], axis=1)
        if (ga.floor.clearance_at(pts) < ga.pass_clear).any():
            continue
        found = ga.escape_for((q[0], q[1], pose[2]), o, max_dist=0.7)
        if found is not None:
            return (float(q[0]), float(q[1])), found[1]
    return None


def rejoin_point(ga, rejoin, from_xy, blockers, clear=0.35, skip=0.0, reach=1.2):
    """양보 로봇이 차선으로 돌아갈 자리 (x, y, yaw).
    원래 자리(rejoin)가 다른 로봇(blockers)에서 clear 이상이면 그대로. 아니면 원래 진행 방향 쪽 차선 중심선 위에서
    모든 blockers 와 clear 이상 떨어지고 from_xy 에서 직선으로 갈 수 있는 가장 가까운 점 (방향은 그 점의 차선 방향).
    skip: 원래 자리에서 이 거리 이상 떨어진 점만 (복귀가 막혀 다시 고를 때 더 멀리). 못 찾으면 None."""
    bl = [np.asarray(b[:2], dtype=float) for b in blockers if b is not None]
    r0 = np.asarray(rejoin[:2], dtype=float)
    if skip <= 0.0 and all(np.hypot(*(r0 - b)) >= clear for b in bl):
        return tuple(rejoin[:3])
    cl = ga.local_centerline(rejoin, reach)
    if not len(cl):
        return None
    h = np.array([math.cos(rejoin[2]), math.sin(rejoin[2])])
    ahead = (cl - r0) @ h
    d0 = np.hypot(*(cl - r0).T)
    ok = (ahead > 0.0) & (d0 >= skip)
    for b in bl:
        ok &= np.hypot(*(cl - b).T) >= clear
    f = np.asarray(from_xy, dtype=float)
    for k in np.argsort(np.where(ok, d0, np.inf)):
        if not ok[k]:
            break
        q = cl[k]
        if ga.wall_between(f, q):
            continue
        # 차선 방향: 근처 중심선 점들의 주성분, 원래 진행 방향과 같은 쪽으로
        near = cl[np.hypot(*(cl - q).T) <= 0.08]
        if len(near) >= 2:
            u = np.linalg.svd(near - near.mean(0))[2][0]
            u = u if u @ h >= 0 else -u
        else:
            u = h
        return float(q[0]), float(q[1]), float(math.atan2(u[1], u[0]))
    return None


def headon(ga, pa, pb, meet_dist=0.6, heading_tol=math.radians(70)):
    """두 차선 로봇이 서로를 향해 마주 보고 meet_dist 안에 있는가 (사이에 벽 없음, 둘 다 차선 위)"""
    d = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
    if d > meet_dist or d < 1e-3:
        return False
    if not (ga.on_lane(pa) and ga.on_lane(pb)) or ga.wall_between(pa, pb):
        return False
    ab = math.atan2(pb[1] - pa[1], pb[0] - pa[0])
    return ang_diff(pa[2], ab) <= heading_tol and ang_diff(pb[2], ab + math.pi) <= heading_tol


def choose_yielder(ga, a, b, tie=0.03, **kw):
    """a, b: dict(id, pose, remaining). '비켜서기 쉬운 쪽이 양보' (사용자 결정 2026-10-07).
    비용 차이가 tie 안이면 남은 거리가 긴 쪽이 양보(목적지에 가까운 쪽 우선), 그래도 같으면 ID 가 큰 쪽이 양보.
    반환 (yielder, priority, escape_xy, info) 또는 둘 다 비킬 자리가 없으면 None."""
    ea = ga.escape_for(a['pose'], b['pose'][:2], **kw)
    eb = ga.escape_for(b['pose'], a['pose'][:2], **kw)
    info = {a['id']: None if ea is None else round(ea[0], 3), b['id']: None if eb is None else round(eb[0], 3)}
    if ea is None and eb is None:
        return None
    if ea is None:
        return b, a, eb[1], dict(info, why='escape only for ' + b['id'])
    if eb is None:
        return a, b, ea[1], dict(info, why='escape only for ' + a['id'])
    if abs(ea[0] - eb[0]) > tie:
        return (a, b, ea[1], dict(info, why='easier escape')) if ea[0] < eb[0] else (b, a, eb[1], dict(info, why='easier escape'))
    ra, rb = a.get('remaining'), b.get('remaining')
    if ra is not None and rb is not None and abs(ra - rb) > 0.05:
        return (a, b, ea[1], dict(info, why='tie: longer remaining yields')) if ra > rb else (b, a, eb[1], dict(info, why='tie: longer remaining yields'))
    return (a, b, ea[1], dict(info, why='tie: id')) if a['id'] > b['id'] else (b, a, eb[1], dict(info, why='tie: id'))


def passed(prio_pose, rejoin, clear_dist=0.45):
    """우선권 로봇이 양보 로봇의 복귀 자리를 충분히 지나갔는가 (멀어졌고, 복귀 자리가 등 뒤)"""
    dx, dy = rejoin[0] - prio_pose[0], rejoin[1] - prio_pose[1]
    return math.hypot(dx, dy) >= clear_dist and dx * math.cos(prio_pose[2]) + dy * math.sin(prio_pose[2]) < 0
