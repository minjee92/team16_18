"""지도(yaml+pgm) → 격자. 계획·간섭 판정용. 행 0 이 월드 y 최소 (이미지와 반대)."""
import os

import numpy as np
import yaml
from scipy import ndimage as ndi


def read_pgm(path):
    raw = open(path, 'rb').read()
    toks, i = [], 0
    while len(toks) < 4:
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        toks.append(raw[i:j])
        i = j
    w, h = int(toks[1]), int(toks[2])
    return np.frombuffer(raw[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


class GridMap:
    def __init__(self, yaml_path, res=0.02):
        meta = yaml.safe_load(open(yaml_path))
        img = read_pgm(os.path.join(os.path.dirname(yaml_path), meta['image']))[::-1]
        p = (255 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
        occ0, free0 = p > meta['occupied_thresh'], p < meta['free_thresh']
        r0 = meta['resolution']
        k = max(1, int(round(res / r0)))
        H, W = occ0.shape[0] // k, occ0.shape[1] // k
        self.res = r0 * k
        self.ox, self.oy = meta['origin'][:2]
        self.occ = occ0[:H * k, :W * k].reshape(H, k, W, k).any(axis=(1, 3))
        self.free = free0[:H * k, :W * k].reshape(H, k, W, k).all(axis=(1, 3)) & ~self.occ
        self.H, self.W = H, W
        # 가장 가까운 비-자유 칸(벽·미확인)까지의 거리 = 여유 (m)
        self.clear = ndi.distance_transform_edt(self.free) * self.res - self.res / 2

    def cell(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def world(self, r, c):
        return np.array([self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res])

    def inside(self, r, c):
        return 0 <= r < self.H and 0 <= c < self.W

    def clearance_at(self, pts):
        pts = np.atleast_2d(pts)
        r = np.clip(((pts[:, 1] - self.oy) / self.res).astype(int), 0, self.H - 1)
        c = np.clip(((pts[:, 0] - self.ox) / self.res).astype(int), 0, self.W - 1)
        return self.clear[r, c]

    def raster(self, pts, radius=0.0):
        """점들이 찍힌 칸 → 그 점들까지의 거리 격자 (m)"""
        m = np.zeros((self.H, self.W), bool)
        for x, y in np.atleast_2d(pts):
            r, c = self.cell(x, y)
            if self.inside(r, c):
                m[r, c] = True
        return ndi.distance_transform_edt(~m) * self.res
