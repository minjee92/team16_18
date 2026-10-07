import math

class CommandGate:
    """Caller holds its publication lock across gate access AND ROS publish.

    An epoch invalidates work started before disable/re-enable/goal changes.
    The deadline is based on capture start, not on when inference completed.
    """
    def __init__(self, max_age: float):
        self.max_age = max_age
        self.enabled = False
        self.epoch = 0
        self.capture_time = float('-inf')
        self.velocity = (0.0, 0.0)

    def invalidate(self):
        self.epoch += 1
        self.capture_time = float('-inf')
        self.velocity = (0.0, 0.0)

    def set_enabled(self, enabled):
        if self.enabled != bool(enabled):
            self.enabled = bool(enabled)
            self.invalidate()

    def submit(self, epoch, capture_time, linear, angular):
        if not self.enabled or epoch != self.epoch:
            return False
        if not all(math.isfinite(v) for v in [capture_time,linear,angular]):
            self.invalidate()
            return False
        self.capture_time = capture_time
        self.velocity = (linear, angular)
        return True

    def current(self, now):
        age = now-self.capture_time
        if not self.enabled or not 0 <= age <= self.max_age:
            return 0.0, 0.0
        return self.velocity
