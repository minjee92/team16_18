"""
[빠름: 모델·ROS 불필요] 갈림길 키 판단 (mj_lane_core.junction.JunctionGuide)

  - ↓ 수동 정지: 바로 대기, 대기 중 무시, 꺾는 중이면 취소 후 대기
  - 대기 조건: 차선 상실 정지 / cross_lane 최근 2번 + 상실 0.5초 초과. cross_lane 한 번만이면 대기 안 함
  - 키는 대기 중에만, odom 없으면 거부
  - 우회전: ENTER(앞으로 enter_dist 0.20 m) → TURN(−ω, 70°) → REACQUIRE(곧게, 좌우 차선 3프레임 연속) → FOLLOW
  - 좌회전은 +ω, 직진은 ENTER·TURN 없이 바로 다시 잡기
  - 너무 많이 돌면(100°, 또는 돌기 2.9초) 정지 + WAIT
  - 제한 시간 넘으면 정지 + WAIT, 비상 정지(abort) 면 WAIT, 꺾는 중 odom 끊기면 WAIT
  - 상태 변화가 events 에 남음
"""

import math

import testlib

from mj_lane_core.junction import ENTER, FOLLOW, REACQUIRE, TURN, WAIT, JunctionGuide

T = testlib.Checker('junction')
DT = 0.1
FOLLOW_CMD = (0.1, 0.02, 'follow')
LOST_CMD = (0.07, 0.0, 'LANE LOST 0.6s')
STOP_CMD = (0.0, 0.0, 'LANE LOST 2.1s STOP')


class Robot:
    """odom 을 명령대로 적분하는 아주 단순한 로봇."""

    def __init__(self):
        self.x = self.y = self.yaw = 0.0

    def step(self, v, w):
        self.yaw += w * DT
        self.x += v * math.cos(self.yaw) * DT
        self.y += v * math.sin(self.yaw) * DT

    @property
    def pose(self):
        return (self.x, self.y, self.yaw)


def wait_by_stop():
    g = JunctionGuide()
    g.update(0.0, STOP_CMD, False, False, False, 2.1, (0, 0, 0))
    return g


# ---------- 대기 조건 ----------
g = JunctionGuide()
out = g.update(0.0, FOLLOW_CMD, True, True, False, 0.0, (0, 0, 0))
T.check('보통 주행은 명령 그대로, 상태 FOLLOW', out == FOLLOW_CMD and g.state == FOLLOW)
g = wait_by_stop()
T.check('차선 상실 정지 → WAIT, 0 명령', g.state == WAIT and g.events[-1]['wait_reason'] == 'LANE LOST STOP')
g = JunctionGuide()
g.update(0.0, FOLLOW_CMD, True, False, True, 0.0, None)          # cross_lane 1번
g.update(0.1, LOST_CMD, False, False, False, 0.6, None)
T.check('cross_lane 한 번 + 상실 0.6초 → 대기 안 함', g.state == FOLLOW)
g.update(0.2, LOST_CMD, False, False, True, 0.7, None)           # 두 번째
out = g.update(0.3, LOST_CMD, False, False, False, 0.8, None)
T.check('cross_lane 1초 안에 2번 + 상실 0.5초 초과 → WAIT', g.state == WAIT and out[:2] == (0.0, 0.0)
        and g.wait_reason == 'CROSS LANE + LOST')
g = JunctionGuide()
g.update(0.0, FOLLOW_CMD, True, False, True, 0.0, None)
g.update(1.5, FOLLOW_CMD, True, False, True, 0.0, None)          # 1.5초 뒤 — 창 밖
g.update(1.6, LOST_CMD, False, False, False, 0.6, None)
T.check('cross_lane 두 번이 1초보다 멀리 떨어지면 대기 안 함', g.state == FOLLOW)

# ---------- 키 ----------
g = JunctionGuide()
ok, msg = g.key('right', 0.0, (0, 0, 0))
T.check('대기 중이 아니면 키 무시', not ok and g.state == FOLLOW, msg)
g = wait_by_stop()
ok, msg = g.key('left', 0.5, None)
T.check('odom 없으면 키 거부, 계속 대기', not ok and g.state == WAIT, msg)


def run_turn(direction, both_from=None, frames=200, timeout=8.0, odom_cut=None):
    """대기에서 키를 누르고 프레임을 돌린다. 반환: (guide, robot, 상태 기록, 명령 기록)."""
    g = JunctionGuide(timeout=timeout)
    r = Robot()
    g.update(0.0, STOP_CMD, False, False, False, 2.1, r.pose)
    g.key(direction, 0.0, r.pose)
    states, cmds = [], []
    for k in range(1, frames):
        now = k * DT
        both = both_from is not None and g.state == REACQUIRE and k >= both_from
        pose = None if odom_cut is not None and k >= odom_cut else r.pose
        v, w, note = g.update(now, (0.08, 0.1, 'follow'), both, both, False, 0.0, pose)
        r.step(v, w)
        states.append(g.state)
        cmds.append((v, w))
        if g.state in (FOLLOW, WAIT):
            break
    return g, r, states, cmds


g, r, states, cmds = run_turn('right', both_from=0)
order = [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]]
T.check('우회전 순서: ENTER → TURN → REACQUIRE → FOLLOW', order == [ENTER, TURN, REACQUIRE, FOLLOW], f'{order}')
T.check('들어가기 거리 약 20 cm 뒤에 돌기 시작', 0.20 <= math.hypot(r.x, r.y) < 0.21, f'{math.hypot(r.x, r.y):.3f} m')
turn_w = [w for s, (_, w) in zip(states, cmds) if s == TURN and w != 0]
turned = -sum(turn_w) * DT
T.check('우회전은 돌기 단계에서 −ω, 약 70° (70–80°)', turn_w and all(w < 0 for w in turn_w) and 70 <= math.degrees(turned) < 80,
        f'{math.degrees(turned):.1f}°')
T.check('끝나면 결과 done 기록', g.events[-1]['kind'] == 'result' and g.events[-1]['result'] == 'done'
        and g.events[-1]['direction'] == 'right')
g, r, states, cmds = run_turn('left', both_from=0)
turn_w = [w for s, (_, w) in zip(states, cmds) if s == TURN and w != 0]
T.check('좌회전은 돌기 단계에서 +ω, 약 70°', turn_w and all(w > 0 for w in turn_w)
        and 70 <= math.degrees(sum(turn_w) * DT) < 80, f'{math.degrees(sum(turn_w) * DT):.1f}°')
g, r, states, cmds = run_turn('straight', both_from=0)
T.check('직진은 들어가기·돌기 없이 바로 다시 잡기 → FOLLOW', states[0] == REACQUIRE and TURN not in states
        and ENTER not in states and states[-1] == FOLLOW)
T.check('다시 잡는 동안은 곧게 (ω 0, 한쪽 차선에 끌려가지 않게)', all(w == 0.0 for s, (_, w) in zip(states, cmds) if s == REACQUIRE))
g, r, states, cmds = run_turn('right', both_from=None)
T.check('차선을 다시 못 잡으면 제한 시간(8초) 뒤 WAIT + timeout', states[-1] == WAIT and len(states) <= 81
        and any(e.get('result') == 'timeout' for e in g.events), f'{len(states)} 프레임')
g, r, states, cmds = run_turn('right', both_from=None, odom_cut=5)
T.check('꺾는 중 odom 끊기면 WAIT', states[-1] == WAIT and any(e.get('result') == 'no odom' for e in g.events))

# 너무 많이 돌기 (몸통이 앞 벽에 닿지 않는다는 0.07 m 기준은 90° 까지 전제)
g = wait_by_stop()
g.key('right', 0.0, (0, 0, 0))
g.update(0.1, FOLLOW_CMD, False, False, False, 0.0, (0.2, 0, 0))          # 들어가기 끝 → 돌기
yaw = 0.0
for k in range(2, 40):
    yaw -= math.radians(5)                                               # 돌기 단계 동안만 5° 씩
    g.update(k * 0.1, FOLLOW_CMD, False, False, False, 0.0, (0.2, 0, yaw))
    if g.state != TURN:
        break
T.check('정상 우회전은 70° 에서 다시 잡기로 (상한 안 걸림)', g.state == REACQUIRE
        and not any(str(e.get('result', '')).startswith('over') for e in g.events), f'{g.state}')
g = wait_by_stop()
g.key('right', 0.0, (0, 0, 0))
g.update(0.1, FOLLOW_CMD, False, False, False, 0.0, (0.2, 0, 0))
g.state = 'REACQUIRE'                                                    # 다시 잡는 중에 방향이 크게 바뀌는 경우
out = g.update(0.3, FOLLOW_CMD, False, False, False, 0.0, (0.2, 0, -math.radians(105)))
T.check('꺾기 시작 방향에서 100° 넘게 돌면 정지 → WAIT (over-rotation angle)', out[:2] == (0.0, 0.0) and g.state == WAIT
        and any(e.get('result') == 'over-rotation (angle)' for e in g.events))
g = wait_by_stop()
g.key('right', 0.0, (0, 0, 0))
g.update(0.1, FOLLOW_CMD, False, False, False, 0.0, (0.2, 0, 0))          # 돌기 시작
k = 2
while g.state == TURN and k < 60:                                        # odom 방향이 멈춰 있음 (회전을 못 따라감)
    g.update(k * 0.1, FOLLOW_CMD, False, False, False, 0.0, (0.2, 0, 0))
    k += 1
T.check('odom 이 회전을 못 따라가면 약 2.9초(100°÷0.6)에 정지 → WAIT (over-rotation time)',
        g.state == WAIT and any(e.get('result') == 'over-rotation (time)' for e in g.events) and 2.9 <= (k - 1) * 0.1 - 0.1 <= 3.1,
        f'{(k - 1) * 0.1 - 0.1:.1f}s')

# 비상 정지
g = wait_by_stop()
g.key('right', 0.0, (0, 0, 0))
g.update(0.1, FOLLOW_CMD, True, True, False, 0.0, (0.01, 0, 0))
g.abort(0.2, 'ESTOP OBSTACLE', (0.01, 0, 0))
T.check('꺾는 중 비상 정지 → WAIT, 결과 aborted', g.state == WAIT and g.wait_reason == 'ESTOP OBSTACLE'
        and any(str(e.get('result', '')).startswith('aborted') for e in g.events))
g = JunctionGuide()
g.abort(0.0, 'ESTOP OBSTACLE')
T.check('꺾는 중이 아니면 abort 는 아무것도 안 함', g.state == FOLLOW and not g.events)
g = JunctionGuide()
ok, msg = g.pause(1.0, (0, 0, 0))
T.check('↓ 수동 정지: FOLLOW 에서 바로 WAIT (이유 MANUAL), 0 명령',
        ok and g.state == WAIT and g.wait_reason == 'MANUAL'
        and g.update(1.1, FOLLOW_CMD, True, True, False, 0.0, (0, 0, 0))[:2] == (0.0, 0.0), msg)
ok, msg = g.pause(1.2)
T.check('↓ 를 대기 중에 또 누르면 무시', not ok and g.state == WAIT, msg)
g = wait_by_stop()
g.key('right', 0.0, (0, 0, 0))
ok, _ = g.pause(0.5, (0.01, 0, 0))
T.check('↓ 를 꺾는 중에 누르면 꺾기 취소 → 대기', ok and g.state == WAIT
        and any(e.get('result') == 'aborted: MANUAL' for e in g.events))
kinds = [e['kind'] for e in wait_by_stop().events]
T.check('대기 시작이 events 에 남음', kinds == ['wait'])

T.finish()
