from typing import Dict, List, Optional
import cv2
import numpy as np
from pinky_lane.config import VISION

MaskMap = Dict[str, List[np.ndarray]]

def segmentation_masks(result, width: int, height: int) -> MaskMap:
    """Use original-image polygons, avoiding padding distortion in raw masks."""
    if result.masks is None or result.boxes is None:
        return {}
    names = result.names
    class_ids = result.boxes.cls.cpu().numpy().astype(int)
    parsed: MaskMap = {}
    for polygon, class_id in zip(result.masks.xy, class_ids):
        polygon = np.asarray(polygon)
        if polygon.ndim != 2 or len(polygon) < 3 or not np.isfinite(polygon).all():
            continue
        mask = np.zeros((height, width), dtype=np.uint8)
        points = np.rint(polygon).astype(np.int32)
        points[:,0] = np.clip(points[:,0], 0, width-1)
        points[:,1] = np.clip(points[:,1], 0, height-1)
        cv2.fillPoly(mask, [points], 1)
        parsed.setdefault(names[class_id], []).append(mask)
    return parsed

def largest_mask(masks: Optional[List[np.ndarray]]) -> Optional[np.ndarray]:
    if not masks:
        return None
    return max(masks, key=lambda item: int(item.sum()))

def largest_area_ratio(masks: Optional[List[np.ndarray]], image_area: int) -> float:
    if not masks or image_area <= 0:
        return 0.0
    return max(float(mask.sum()) for mask in masks) / float(image_area)

class LaneSegmenter:
    """Load the segmentation model only when hardware starts."""
    def __init__(self):
        from ultralytics import YOLO
        self.model = YOLO(VISION.model_path)
        names = self.model.names
        found = set(names.values() if isinstance(names, dict) else names)
        missing = {"left_lane", "right_lane", "crosswalk"} - found
        if missing:
            raise ValueError(f"Missing segmentation classes: {sorted(missing)}")

    def predict(self, frame):
        return self.model.predict(source=frame, conf=VISION.confidence,
                                  imgsz=VISION.infer_size, verbose=False)[0]
