"""Edit configuration here, then restart the program."""
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MAPS_DIR = PROJECT_ROOT / "maps"
MODELS_DIR = PROJECT_ROOT / "models"
DRIVE_OUTPUT = False

@dataclass(frozen=True)
class VisionConfig:
    model_path: str = "best_ncnn_model"
    frame_width: int = 640
    frame_height: int = 480
    infer_size: int = 320
    confidence: float = 0.50
    rotate_180: bool = True
    mirror_input: bool = False
    swap_rb: bool = False
    use_lcd: bool = True
    lcd_every_n_frames: int = 2

@dataclass(frozen=True)
class LaneConfig:
    sample_rows: Tuple[float, ...] = (0.90, 0.80, 0.70, 0.60)
    row_weights: Tuple[float, ...] = (0.40, 0.30, 0.20, 0.10)
    sample_band_half_height: int = 2

    initial_half_width_ratio: float = 0.28
    half_width_alpha: float = 0.10
    min_lane_width_ratio: float = 0.18
    max_lane_width_ratio: float = 0.85

    kp: float = 1.00
    kd: float = 0.18
    steer_alpha: float = 0.35
    target_alpha: float = 0.35

    min_valid_rows: int = 2
    lost_slow_s: float = 0.5
    lost_stop_s: float = 2.0

@dataclass(frozen=True)
class MissionConfig:
    map_frame: str = "map"
    robot_frame: str = "base_footprint"

    turn_threshold_deg: float = 50.0
    allow_left_turn: bool = False
    goal_radius_m: float = 0.25

    crosswalk_enter_ratio: float = 0.020
    crosswalk_release_ratio: float = 0.008
    crosswalk_confirm_frames: int = 3
    intersection_cooldown_s: float = 5.0

    enter_distance_m: float = 0.18
    enter_timeout_s: float = 6.0
    turn_angle_deg: float = 90.0
    turn_tolerance_deg: float = 7.0
    turn_timeout_s: float = 8.0

@dataclass(frozen=True)
class MotionConfig:
    control_hz: float = 10.0
    start_delay_s: float = 1.0

    base_speed: float = 0.10
    min_speed: float = 0.05
    corner_slowdown: float = 0.50

    lane_angular_gain: float = 2.0
    max_angular: float = 1.50

    turn_forward_speed: float = 0.015
    turn_rate_max: float = 1.20
    turn_rate_min: float = 0.45
    turn_kp: float = 1.40

    odom_stale_s: float = 0.70
    vision_stale_s: float = 0.70
    map_stale_s: float = 2.0

VISION = VisionConfig(model_path=str(MODELS_DIR / "best_ncnn_model"))
LANE = LaneConfig()
MISSION = MissionConfig()
MOTION = MotionConfig()
