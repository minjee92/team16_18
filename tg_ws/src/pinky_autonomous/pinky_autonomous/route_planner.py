"""갈림길 출구 선택: SLAM 점유격자에서 '출발점까지 가장 싼 방향'(좌/우/직진/후진)을 고른다.

ROS에 의존하지 않는다(numpy, cv2, heapq). 로봇이 직접 주행 경로를 따라가는 용도가 아니라,
갈림길에서 어느 쪽으로 꺾을지만 정하는 용도다. 이후 실제 주행은 차선 추종이 맡는다.

비용 모델
  - 점유 칸(벽)과 그 팽창 영역: 통과 불가
  - 알려진 빈 칸: 1.0 / 칸
  - 미탐색 칸(-1): unknown_cost / 칸  (로봇이 가보지 않은 쪽은 비싸게 쳐서, 벽을 뚫는 지름길 대신
                                      이미 지나온 길을 더 선호하게 한다)
"""

import heapq
import math

import cv2
import numpy as np

UNKNOWN = -1
OCC_THRESH = 65

_NEIGH = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
          (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414))

LOOKAHEAD_M = 0.7          # 최단 경로를 이만큼 따라간 지점의 방향으로 좌/우/직진/후진을 가른다
SIDE_MIN_DEG = 30.0        # 경로 방향이 로봇 정면 기준 ±30° 안이면 직진, 150° 밖이면 후진, 그 사이는 좌/우


def _crop_to_known(grid, margin):
    ys, xs = np.nonzero(grid != UNKNOWN)
    if ys.size == 0:
        return None
    y0, y1 = max(ys.min() - margin, 0), min(ys.max() + margin + 1, grid.shape[0])
    x0, x1 = max(xs.min() - margin, 0), min(xs.max() + margin + 1, grid.shape[1])
    return y0, x0, grid[y0:y1, x0:x1]


def build_cost(grid, resolution, unknown_cost, inflate_m):
    occ = grid >= OCC_THRESH
    k = max(1, int(round(inflate_m / resolution)))
    occ_inflated = cv2.dilate(occ.astype(np.uint8), np.ones((2 * k + 1, 2 * k + 1), np.uint8)).astype(bool)
    cost = np.where(grid == UNKNOWN, unknown_cost, 1.0).astype(np.float64)
    cost[occ_inflated] = np.inf
    return cost, occ


def _clear_inflation(cost, occ, cell, radius_cells, base):
    """로봇/출발점 주변은 팽창 때문에 갇히지 않게 열어 둔다 (실제 벽 칸은 그대로 막힘)."""
    h, w = cost.shape
    cy, cx = cell
    y0, y1, x0, x1 = max(cy - radius_cells, 0), min(cy + radius_cells + 1, h), max(cx - radius_cells, 0), min(cx + radius_cells + 1, w)
    yy, xx = np.ogrid[y0:y1, x0:x1]
    disk = (yy - cy) ** 2 + (xx - cx) ** 2 <= radius_cells ** 2
    region = cost[y0:y1, x0:x1]
    open_mask = disk & ~occ[y0:y1, x0:x1]
    region[open_mask] = base[y0:y1, x0:x1][open_mask]


def dijkstra(cost, src):
    """src 칸에서 모든 칸까지의 누적 비용(칸 단위). 통과 불가 칸은 inf."""
    h, w = cost.shape
    c = cost.tolist()
    inf = math.inf
    dist = [[inf] * w for _ in range(h)]
    sy, sx = src
    dist[sy][sx] = 0.0
    pq = [(0.0, sy, sx)]
    push, pop = heapq.heappush, heapq.heappop
    while pq:
        d, y, x = pop(pq)
        if d > dist[y][x]:
            continue
        c0 = c[y][x]
        for dy, dx, step in _NEIGH:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w:
                c1 = c[ny][nx]
                if c1 == inf:
                    continue
                nd = d + step * 0.5 * (c0 + c1)
                if nd < dist[ny][nx]:
                    dist[ny][nx] = nd
                    push(pq, (nd, ny, nx))
    return np.array(dist)


def _bucket(rel):
    """로봇 정면 기준 각도(rad, 왼쪽이 +) → 방향 이름."""
    deg = math.degrees(rel)
    if abs(deg) < SIDE_MIN_DEG:
        return 'straight'
    if abs(deg) > 180.0 - SIDE_MIN_DEG:
        return 'back'
    return 'left' if deg > 0 else 'right'


def choose_exit(grid, resolution, origin_xy, robot_pose, start_xy,
                unknown_cost=2.0, inflate_m=0.08):
    """출발점까지의 최단 경로가 로봇 위치에서 어느 쪽으로 뻗는지 구한다.

    grid        : (H, W) int8 점유격자 (-1 미탐색, 0 빈 칸, 100 점유), 행=y, 열=x
    origin_xy   : 격자 (0,0) 칸의 map 좌표(m)
    robot_pose  : (x, y, yaw) in map
    start_xy    : 출발 위치 (x, y) in map
    반환: (방향 'left'/'right'/'straight'/'back' 또는 None, 정보 dict)
    정보: cost_m(출발점까지 비용, 미탐색 가중 포함), rel_deg(경로 방향, 정면 기준 왼쪽 +), unknown_ratio(경로 중 미탐색 비율)
    """
    cropped = _crop_to_known(grid, margin=int(round(0.5 / resolution)))
    if cropped is None:
        return None, {}
    oy, ox, sub = cropped

    def to_cell(x, y):
        return (int(math.floor((y - origin_xy[1]) / resolution)) - oy,
                int(math.floor((x - origin_xy[0]) / resolution)) - ox)

    def in_bounds(cell):
        return 0 <= cell[0] < sub.shape[0] and 0 <= cell[1] < sub.shape[1]

    cost, occ = build_cost(sub, resolution, unknown_cost, inflate_m)
    base = np.where(sub == UNKNOWN, unknown_cost, 1.0)

    rx, ry, yaw = robot_pose
    robot_cell, start_cell = to_cell(rx, ry), to_cell(*start_xy)
    if not in_bounds(robot_cell) or not in_bounds(start_cell):
        return None, {}
    r_open = int(round((inflate_m + 0.07) / resolution))
    for cell in (robot_cell, start_cell):
        _clear_inflation(cost, occ, cell, r_open, base)
    if occ[start_cell] or occ[robot_cell]:
        return None, {}

    dist = dijkstra(cost, start_cell)
    if not math.isfinite(dist[robot_cell]):
        return None, {'cost_m': math.inf}

    # 로봇 칸에서 비용이 줄어드는 쪽으로 따라가며 LOOKAHEAD_M 만큼 진행한 지점을 구한다
    y, x = robot_cell
    unknown_steps = steps = 0
    h, w = dist.shape
    while True:
        if math.hypot(y - robot_cell[0], x - robot_cell[1]) * resolution >= LOOKAHEAD_M or dist[y, x] == 0:
            break
        best, bd = None, dist[y, x]
        for dy, dx, _ in _NEIGH:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and dist[ny, nx] < bd:
                best, bd = (ny, nx), dist[ny, nx]
        if best is None:
            break
        y, x = best
        steps += 1
        unknown_steps += int(sub[y, x] == UNKNOWN)
    if (y, x) == robot_cell:
        return None, {'cost_m': float(dist[robot_cell] * resolution)}

    heading = math.atan2((y - robot_cell[0]), (x - robot_cell[1]))      # 격자 행=y, 열=x → map 좌표계와 같은 방향
    rel = (heading - yaw + math.pi) % (2.0 * math.pi) - math.pi
    info = {'cost_m': float(dist[robot_cell] * resolution), 'rel_deg': math.degrees(rel),
            'unknown_ratio': unknown_steps / max(steps, 1)}
    return _bucket(rel), info
