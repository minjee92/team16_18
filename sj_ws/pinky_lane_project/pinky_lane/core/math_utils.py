import math

def wrap_angle(angle: float) -> float:
    """Angle -> [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def quaternion_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
