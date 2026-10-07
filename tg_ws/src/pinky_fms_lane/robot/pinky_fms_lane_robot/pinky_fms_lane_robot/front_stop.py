"""전방 초음파 정지 판정 (ROS 없음). cmd_vel_gate 가 쓴다.

원본: mj_ws/src/robot/drive_control.py 의 FrontStop (사용자 코드, 브랜치 feat/mj-lane-seg 커밋 5833b69, 무효값 3회 연속 규칙).
수정 없이 클래스만 복사했다. 게이트는 latch(check/clear, 사람이 풀어야 다시 움직임)를 쓰지 않고 reason() 만 매 주기 본다.
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
