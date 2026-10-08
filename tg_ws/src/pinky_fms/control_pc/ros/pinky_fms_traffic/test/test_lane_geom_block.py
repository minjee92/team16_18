"""lane_geom 막을 영역(block_rects) 시험: 지도 왼쪽 벽 아래 출입구 틈을 계산에서만 막으면 아레나 밖 비켜설 자리가 후보에서 빠진다."""
import os

import numpy as np
import pytest

from pinky_fms_traffic.lane_geom import LaneGeometry

MAPS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'backend', 'maps')
LANES = os.path.join(MAPS, 'mission4_3_lanes_1cm', 'map.yaml')
FLOOR = os.path.join(MAPS, 'mission4_3_nolanes_1cm', 'map.yaml')
GAP = (-0.14, -0.07, -0.12, 0.17)                 # run_real.sh 의 ESCAPE_BLOCK 기본값과 같다
L, R, B, T = -0.117, 2.183, -0.056, 1.144         # 아레나 벽 안쪽 면 (줄자 파일)


def outside(xy):
    return (xy[:, 0] < L) | (xy[:, 0] > R) | (xy[:, 1] < B) | (xy[:, 1] > T)


@pytest.mark.parametrize('margin', [0.03, 0.08])
def test_block_removes_outside_candidates(margin):
    before = LaneGeometry(LANES, FLOOR, wall_margin=margin)
    after = LaneGeometry(LANES, FLOOR, wall_margin=margin, block_rects=[GAP])
    assert outside(before.esc_xy).sum() > 0               # 막지 않으면 틈 밖 바닥도 후보 (예전 동작)
    assert outside(after.esc_xy).sum() == 0               # 막으면 아레나 밖 후보 없음
    inside_before = before.esc_xy[~outside(before.esc_xy)]
    assert len(after.esc_xy) >= 0.95 * len(inside_before)  # 아레나 안 후보는 거의 그대로


def test_block_seals_wall_between():
    g = LaneGeometry(LANES, FLOOR, block_rects=[GAP])
    assert g.wall_between((0.05, 0.05), (-0.30, 0.05))     # 틈을 지나는 직선은 벽에 막힌다
    assert not LaneGeometry(LANES, FLOOR).wall_between((0.05, 0.05), (-0.30, 0.05))


def test_escape_near_gap_stays_inside():
    g = LaneGeometry(LANES, FLOOR, block_rects=[GAP])
    for pose, other in [((0.10, 0.24, np.pi), (0.40, 0.24)), ((0.07, 0.30, -np.pi / 2), (0.07, 0.60))]:
        for max_d in (0.45, 0.7):
            r = g.escape_for(pose, other, max_dist=max_d)
            if r is not None:
                assert not outside(np.array([r[1]]))[0], r
