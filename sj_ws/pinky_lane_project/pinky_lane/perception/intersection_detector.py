from pinky_lane.config import MISSION

class IntersectionDetector:
    """Crosswalk mask가 여러 프레임 유지될 때 한 번의 교차로 이벤트로 만든다."""

    def __init__(self) -> None:
        self._hits = 0
        self._active = False
        self._last_event = -1e9

    def reset(self) -> None:
        self._hits = 0
        self._active = False
        self._last_event = -1e9

    def update(self, area_ratio: float, now: float) -> bool:
        if self._active:
            if area_ratio <= MISSION.crosswalk_release_ratio:
                self._active = False
                self._hits = 0
            return False

        if area_ratio >= MISSION.crosswalk_enter_ratio:
            self._hits += 1
        else:
            self._hits = 0

        confirmed = self._hits >= MISSION.crosswalk_confirm_frames
        cooled_down = now - self._last_event >= MISSION.intersection_cooldown_s

        if confirmed and cooled_down:
            self._active = True
            self._hits = 0
            self._last_event = now
            return True

        return False
