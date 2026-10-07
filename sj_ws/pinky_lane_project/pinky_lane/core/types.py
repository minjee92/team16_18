from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional

@dataclass
class LanePoint:
    x: int
    y: int
    source: str  # both | left | right

@dataclass
class LaneEstimate:
    points: List[LanePoint]
    steering: float
    confidence: float
    valid: bool
    lost_frames: int
    target_x: Optional[float]
    lost_seconds: float = 0.0
    can_coast: bool = False

class DriveState(Enum):
    IDLE = auto()
    FOLLOW = auto()
    ENTER_INTERSECTION = auto()
    TURNING = auto()
    ARRIVED = auto()
    FAULT = auto()

class TurnChoice(Enum):
    STRAIGHT = auto()
    LEFT = auto()
    RIGHT = auto()
