"""
갈림길에서 사람이 고른 방향으로 꺾기 (ROS·카메라 무관)

주행 노드가 --junction-keys 일 때만 만든다. 입력·출력은 모두 인자로 주고받는다 (시각·odom·검출 결과).

상태:
  FOLLOW     보통 차선 추종 (LaneFollowController 결과를 그대로 낸다)
  WAIT       갈림길 대기: 정지하고 키(← → ↑)를 기다린다
  ENTER      들어가기: 천천히 enter_dist 만큼 앞으로 (odom 거리). 직진(↑)은 이 단계 없이 바로 REACQUIRE
  TURN       돌기: 제자리에서 odom 방향이 turn_angle 만큼 바뀔 때까지 (좌 +ω, 우 −ω)
  REACQUIRE  다시 잡기: 천천히 곧게 가며 좌우 차선이 둘 다 보이는 프레임이 reacquire_frames 번 연속이면 FOLLOW
             (한쪽 차선만 보일 때 조향하면 그 선 쪽으로 끌려가 차선 밖으로 나갔다 — 시뮬 D2 확인. 그래서 곧게만 간다)

대기에 들어가는 조건 (FOLLOW 에서):
  - 최근 cross_window 초 안에 cross_lane 이 cross_min 번 이상 보였고, 차선 상실이 wait_lost_sec 를 넘음
  - 또는 차선 상실 정지 (controller 가 STOP 을 냄)

안전:
  - 초음파 비상 정지는 주행 노드가 맨 마지막에 0 을 낸다 (이 판단보다 우선). 꺾는 중에 비상 정지가 걸리면
    abort() 로 꺾기를 취소하고 WAIT 로 돌아가 다시 키를 받는다 (멈춘 동안 위치가 바뀌었을 수 있어서)
  - 꺾기 전체가 timeout 초(시각 원천 기준) 안에 안 끝나면 정지하고 WAIT 로 돌아간다
  - odom 이 없으면 꺾기를 시작하지 않는다 (각도·거리를 잴 수 없음)

각 상태 변화는 events 목록에 남긴다 (주행 노드가 갈림길 CSV 에 쓴다).
"""

import math
from collections import deque

FOLLOW, WAIT, ENTER, TURN, REACQUIRE = 'FOLLOW', 'WAIT', 'ENTER', 'TURN', 'REACQUIRE'
DIRECTIONS = {'left': +1, 'right': -1, 'straight': 0}


def angle_diff(a, b):
    """a − b 를 −π..π 로."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class JunctionGuide:
    def __init__(self, enter_dist=0.20, enter_speed=0.05, turn_rate=0.6, turn_angle_deg=70.0,
                 reacquire_speed=0.05, reacquire_frames=3, timeout=8.0,
                 cross_window=1.0, cross_min=2, wait_lost_sec=0.5):
        self.enter_dist = enter_dist
        self.enter_speed = enter_speed
        self.turn_rate = turn_rate
        self.turn_angle = math.radians(turn_angle_deg)
        self.reacquire_speed = reacquire_speed
        self.reacquire_frames = reacquire_frames
        self.timeout = timeout
        self.cross_window = cross_window
        self.cross_min = cross_min
        self.wait_lost_sec = wait_lost_sec

        self.state = FOLLOW
        self.direction = None
        self.wait_reason = ''
        self.wait_since = None
        self.start_time = None
        self.start_pose = None
        self.both_streak = 0
        self.cross_seen = deque()
        self.events = []          # [{'kind', 'time', ...}] 주행 노드가 꺼내 간다

    # ---------- 주행 노드에서 부름 ----------

    def update(self, now, command, valid, both_lanes, cross_lane, lost_time, pose):
        """
        이번 프레임의 명령을 정한다.
          command    : LaneFollowController.command() 결과 (v, w, note)
          valid      : 차선을 하나라도 봤는지, both_lanes: 좌우 둘 다 봤는지
          cross_lane : 이번 프레임에 cross_lane 이 검출됐는지
          pose       : odom (x, y, yaw) 또는 None
        반환: (v, w, note)
        """
        v, w, note = command
        if cross_lane:
            self.cross_seen.append(now)
        while self.cross_seen and now - self.cross_seen[0] > self.cross_window:
            self.cross_seen.popleft()

        if self.state == FOLLOW:
            if note.endswith('STOP'):
                self._enter_wait(now, 'LANE LOST STOP')
            elif not valid and lost_time > self.wait_lost_sec and len(self.cross_seen) >= self.cross_min:
                self._enter_wait(now, 'CROSS LANE + LOST')
            else:
                return v, w, note

        if self.state == WAIT:
            return 0.0, 0.0, 'JUNCTION WAIT (← → ↑)'

        if now - self.start_time > self.timeout:
            self._finish(now, 'timeout', pose)
            self._enter_wait(now, 'TIMEOUT')
            return 0.0, 0.0, 'JUNCTION TIMEOUT'
        if pose is None:
            self._finish(now, 'no odom', pose)
            self._enter_wait(now, 'NO ODOM')
            return 0.0, 0.0, 'JUNCTION NO ODOM'

        x0, y0, yaw0 = self.start_pose
        x, y, yaw = pose
        if self.state == ENTER:
            if math.hypot(x - x0, y - y0) >= self.enter_dist:
                self.state = TURN
                self.start_pose = pose
                return 0.0, 0.0, 'JUNCTION TURN'
            return self.enter_speed, 0.0, 'JUNCTION ENTER'

        if self.state == TURN:
            if abs(angle_diff(yaw, yaw0)) >= self.turn_angle:
                self.state = REACQUIRE
                self.both_streak = 0
                return 0.0, 0.0, 'JUNCTION REACQUIRE'
            return 0.0, DIRECTIONS[self.direction] * self.turn_rate, f'JUNCTION TURN {self.direction}'

        # REACQUIRE
        self.both_streak = self.both_streak + 1 if both_lanes else 0
        if self.both_streak >= self.reacquire_frames:
            self._finish(now, 'done', pose)
            self.state = FOLLOW
            return v, w, note
        return self.reacquire_speed, 0.0, 'JUNCTION REACQUIRE'

    def key(self, direction, now, pose):
        """화살표 키. 대기 중에만 받는다. 반환: (받았는지, 안내 글)."""
        if direction not in DIRECTIONS:
            return False, f'모르는 방향: {direction}'
        if self.state != WAIT:
            return False, f'갈림길 대기 중이 아니라 무시 ({self.state})'
        if pose is None:
            return False, 'odom 이 없어 꺾을 수 없음 (odom 토픽 확인)'
        self.events.append({'kind': 'key', 'time': now, 'direction': direction, 'wait_reason': self.wait_reason,
                            'wait_sec': now - self.wait_since, 'pose': pose})
        self.direction = direction
        self.start_time = now
        self.start_pose = pose
        self.both_streak = 0
        self.state = REACQUIRE if direction == 'straight' else ENTER
        return True, f'{direction} 시작'

    def abort(self, now, reason, pose=None):
        """비상 정지 등으로 꺾기를 그만둔다 → WAIT. 꺾는 중이 아니면 아무것도 안 함."""
        if self.state in (ENTER, TURN, REACQUIRE):
            self._finish(now, f'aborted: {reason}', pose)
            self._enter_wait(now, reason)

    @property
    def maneuvering(self):
        return self.state in (ENTER, TURN, REACQUIRE)

    # ---------- 내부 ----------

    def _enter_wait(self, now, reason):
        self.state = WAIT
        self.wait_reason = reason
        self.wait_since = now
        self.direction = None
        self.events.append({'kind': 'wait', 'time': now, 'wait_reason': reason})

    def _finish(self, now, result, pose):
        self.events.append({'kind': 'result', 'time': now, 'direction': self.direction, 'result': result,
                            'maneuver_sec': now - self.start_time, 'pose': pose})
