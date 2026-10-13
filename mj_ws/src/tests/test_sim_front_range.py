"""
[빠름: ROS 환경 필요] 시뮬 전용 전방 초음파 흉내(mj_lane_sim/scripts/sim_front_range.py)의 계산

라이다 점 → 초음파 위치에서 본 전방 거리. 빔(±0.13 rad) 밖·뒤쪽·NaN 은 무시하고, 없으면 실물 상한 0.97 m.
"""

import math
import sys

import testlib

sys.path.insert(0, str(testlib.SRC_DIR / 'mj_lane_sim' / 'scripts'))
import sim_front_range as SFR     # noqa: E402

T = testlib.Checker('sim_front_range')

N = 640
angle_min, inc = -math.pi, 2 * math.pi / N


def scan_with(points):
    """points: [(base_link 기준 각도(rad), 라이다에서 거리)] → 라이다 거리 목록 (나머지는 inf)."""
    ranges = [math.inf] * N
    for base_angle, r in points:
        lidar_angle = math.remainder(base_angle - SFR.LIDAR_YAW, 2 * math.pi)
        ranges[round((lidar_angle - angle_min) / inc) % N] = r
    return ranges


def fr(points):
    return SFR.front_range(scan_with(points), angle_min, inc, 0.05, 12.0)


offset = SFR.SONAR_X - SFR.LIDAR_X                        # 라이다와 초음파의 앞뒤 거리 (0.0437 m)
T.check('정면 0.5 m 점 → 초음파 기준 거리', abs(fr([(0.0, 0.5)]) - (0.5 - offset)) < 1e-6, f'{fr([(0.0, 0.5)]):.4f}')
T.check('아무것도 없음 → 실물 상한 0.97 m', fr([]) == SFR.SENSOR_CAP)
T.check('멀리 있음(2 m) → 0.97 m 로 자름', fr([(0.0, 2.0)]) == SFR.SENSOR_CAP)
T.check('뒤쪽 점은 무시', fr([(math.pi, 0.2)]) == SFR.SENSOR_CAP)
T.check('빔 밖(옆 30°) 점은 무시', fr([(math.radians(30), 0.3)]) == SFR.SENSOR_CAP)
T.check('빔 안(5°) 점은 거리로', fr([(math.radians(5), 0.3)]) < 0.3)
T.check('가장 가까운 점을 고름', abs(fr([(0.0, 0.6), (math.radians(3), 0.3)]) - fr([(math.radians(3), 0.3)])) < 1e-9)
nan_scan = scan_with([(0.0, 0.5)])
nan_scan[1] = math.nan
T.check('NaN 은 무시', SFR.front_range(nan_scan, angle_min, inc, 0.05, 12.0) == fr([(0.0, 0.5)]))

T.finish()
