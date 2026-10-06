"""
2단계 주행 판단 (ROS·카메라 무관)

차선 추적 결과와 전방 초음파 거리로 cmd_vel 값(v, ω)을 정한다.
ROS 에 의존하지 않으므로 PC에서 단위 테스트·저장 영상 재생 테스트를 할 수 있다.

  FrontStop           : 전방 초음파 비상 정지 판정 (fail-safe, 수동 재개)
  LaneFollowController: 차선 추종 속도·각속도와 차선 상실 시 단계적 대응

시각(now)은 모두 초 단위 단조 시계 (로봇은 time.monotonic()).
"""

import math


class FrontStop:
    """
    전방 초음파 비상 정지. 아래 중 하나면 정지한다 (fail-safe):

      - 측정값이 한 번도 안 옴, 또는 아직 유효한 거리가 없음 → 'NO SONAR'   (pinky_sensor_adc 미실행 등)
      - 마지막 측정이 timeout 초보다 오래됨                → 'SONAR TIMEOUT' (노드 중단)
      - 무효값이 invalid_count 회 연속                     → 'SONAR INVALID'
          무효값 = range < min_range 또는 NaN/inf. I2C 실패 시 드라이버가 −0.03 m 를 보낸다.
          1~2회 단발 튐은 무시하고 횟수만 센다 (한 번에 정지하면 주행이 몇 초마다 끊김).
          연속이면 센서가 실제로 나빠진 것이므로 정지. NaN 은 비교가 항상 거짓이라 따로 막는다
      - 마지막 유효 거리 ≤ stop_distance                    → 'OBSTACLE'
          (튐이 끼어 있어도 장애물 판정은 마지막 유효 거리로 한다)

    한 번 걸리면 clear() 로 풀 때까지 정지 상태를 유지한다 (수동 재개).
    clear() 는 지금 정지 조건이 하나도 없을 때만 성공한다.

    무효값 통계: invalid_ignored (무시한 무효 측정 수), invalid_spikes (무시한 튐 구간 수),
    invalid_stops (무효값으로 정지한 횟수). 정지로 이어진 무효값은 무시 수에 넣지 않는다.
    """

    def __init__(self, stop_distance, timeout, min_range, invalid_count=3):
        self.stop_distance = stop_distance
        self.timeout = timeout
        self.min_range = min_range
        self.invalid_count = invalid_count

        self.last_range = None        # 마지막 측정 (무효값 포함, 로그용)
        self.valid_range = None       # 마지막 유효 거리 (장애물 판정에 씀)
        self.last_time = None         # 마지막 측정 수신 시각 (무효값도 센서가 살아 있다는 신호)
        self.invalid_streak = 0
        self.invalid_ignored = 0
        self.invalid_spikes = 0
        self.invalid_stops = 0
        self.tripped = False
        self.trip_reason = ''

    def on_range(self, range_m, now):
        """초음파 측정 하나를 기록한다 (m, 수신 시각)."""
        value = float(range_m)
        self.last_range = value
        self.last_time = now
        if not math.isfinite(value) or value < self.min_range:
            self.invalid_streak += 1
            return
        if 0 < self.invalid_streak < self.invalid_count:     # 정지까지 가지 않고 끝난 튐 → 무시한 것으로 확정
            self.invalid_ignored += self.invalid_streak
            self.invalid_spikes += 1
        self.invalid_streak = 0
        self.valid_range = value

    def reason(self, now):
        """지금 정지해야 하는 이유. 없으면 ''."""
        if self.last_time is None:
            return 'NO SONAR'
        if now - self.last_time > self.timeout:
            return 'SONAR TIMEOUT'
        if self.invalid_streak >= self.invalid_count:
            return 'SONAR INVALID'
        if self.valid_range is None:
            return 'NO SONAR'
        if self.valid_range <= self.stop_distance:
            return 'OBSTACLE'
        return ''

    def check(self, now):
        """정지 조건을 평가해 걸리면 정지 상태로 만든다. 반환: 지금 정지 상태인지."""
        reason = self.reason(now)
        if reason and not self.tripped:
            self.tripped = True
            self.trip_reason = reason
            if reason == 'SONAR INVALID':
                self.invalid_stops += 1
        return self.tripped

    def clear(self, now):
        """수동 재개. 지금 정지 조건이 없을 때만 풀린다. 반환: (성공 여부, 남은 이유)."""
        reason = self.reason(now)
        if reason:
            return False, reason
        self.tripped = False
        self.trip_reason = ''
        return True, ''

    def invalid_summary(self):
        """(무시한 무효 측정 수, 무시한 튐 구간 수, 무효로 정지한 횟수). 끝나지 않은 짧은 연속도 무시로 센다."""
        pending = self.invalid_streak if 0 < self.invalid_streak < self.invalid_count else 0
        return self.invalid_ignored + pending, self.invalid_spikes + (1 if pending else 0), self.invalid_stops


class LaneFollowController:
    """
    차선 추적 결과(steer, valid, lost_time) → (v, ω, note)

    차선을 볼 때: 조향이 클수록 감속 (corner_slowdown), 최소 min_speed
    차선을 못 볼 때 (lost_time 기준, robot/lane_mission_drive.py 의 단계적 대응을 초 단위로):
      0 < t ≤ lost_slow_sec   : base_speed × lost_speed_ratio, 직전 조향 유지
      t > lost_slow_sec       : min_speed, 직전 조향 유지
      t > lost_stop_sec       : 정지
    ω 는 ±max_angular 로 제한한다. 부호: steer > 0 (목표가 오른쪽) → ω < 0 (우회전).
    """

    def __init__(self, base_speed, min_speed, max_angular, corner_slowdown,
                 steer_to_angular, lost_slow_sec, lost_stop_sec, lost_speed_ratio=0.7):
        self.base_speed = base_speed
        self.min_speed = min_speed
        self.max_angular = max_angular
        self.corner_slowdown = corner_slowdown
        self.steer_to_angular = steer_to_angular
        self.lost_slow_sec = lost_slow_sec
        self.lost_stop_sec = lost_stop_sec
        self.lost_speed_ratio = lost_speed_ratio

    def command(self, steer, valid, lost_time):
        angular = -self.steer_to_angular * steer
        angular = max(-self.max_angular, min(self.max_angular, angular))

        if not valid:
            if lost_time > self.lost_stop_sec:
                return 0.0, 0.0, f'LANE LOST {lost_time:.1f}s STOP'
            if lost_time > self.lost_slow_sec:
                return self.min_speed, angular, f'LANE LOST {lost_time:.1f}s SLOW'
            return self.base_speed * self.lost_speed_ratio, angular, f'LANE LOST {lost_time:.1f}s'

        magnitude = min(abs(steer), 1.0)
        speed = max(self.base_speed * (1.0 - self.corner_slowdown * magnitude), self.min_speed)
        return speed, angular, ''
