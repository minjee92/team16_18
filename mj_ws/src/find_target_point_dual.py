import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent   # mj_ws/

# =========================================================================
# [설정]
INPUT_SOURCE = str(BASE_DIR / 'inputs' / 'pinky_20260919_182009.mp4')  # 웹캠은 0, 저장된 동영상 검증은 inputs/ 아래 파일
# =========================================================================

# 방금 제대로 구워진 4-Class Segmentation 모델 경로로 수정
MODEL_PATH = str(BASE_DIR / 'models' / '260928_yolon_best.pt')
OUTPUT_VIDEO = str(BASE_DIR / 'outputs' / 'result_find_target_point_dual.mp4')

# 차선 추종을 위한 알고리즘 설정값
Y_LOOKAHEAD = 350       # 타겟 중심점을 찾을 고정 Y 좌표 (로봇의 시야거리)
LANE_WIDTH_HALF = 180   # 도로 폭의 절반 픽셀값 (한쪽 차선만 보일 때 사용할 가상의 차선 폭)

# 1. 모델 로드
model = YOLO(MODEL_PATH)
class_names = model.names # {0: 'cross_lane', 1: 'crosswalk', 2: 'left_lane', 3: 'right_lane'}

# 2. 비디오 캡처 설정
cap = cv2.VideoCapture(INPUT_SOURCE)
if not cap.isOpened():
    print(f"오류: 입력 소스를 열 수 없습니다 -> {INPUT_SOURCE}")
    exit()

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
fps = fps if fps > 0 else 30.0

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (width, height))

print("자율주행 4-Class 인식 및 Target Point 계산 시작... (종료: 'q')")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 3. 추론 실행 (Segmentation)
    results = model.predict(source=frame, conf=0.5, verbose=False)
    result = results[0]
    
    annotated_frame = result.plot()

    left_centroid = None
    right_centroid = None

    # 4. 객체별 마스크 중심점 추출 로직
    if result.masks is not None and result.boxes is not None:
        masks_xy = result.masks.xy 
        classes = result.boxes.cls.cpu().numpy() 

        for mask_pts, cls_id in zip(masks_xy, classes):
            cls_name = class_names[int(cls_id)] 
            if len(mask_pts) == 0:
                continue
                
            pts = np.array(mask_pts, dtype=np.int32)

            # [A] 좌/우 차선 인식 로직 (시야거리 Y_LOOKAHEAD 기준 위치 고정)
            if cls_name in ['left_lane', 'right_lane']:
                target_y_pts = [p[0] for p in pts if Y_LOOKAHEAD - 30 < p[1] < Y_LOOKAHEAD + 30]
                
                if len(target_y_pts) > 0:
                    cx = int(np.mean(target_y_pts))
                    cy = Y_LOOKAHEAD
                else:
                    M = cv2.moments(pts)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                    else:
                        continue
                
                # 모델이 판별한 클래스(좌/우)를 전적으로 신뢰
                if cls_name == 'left_lane':
                    left_centroid = (cx, cy)
                    cv2.circle(annotated_frame, left_centroid, 7, (0, 255, 255), -1)
                    cv2.putText(annotated_frame, "LEFT LANE", (cx - 20, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                elif cls_name == 'right_lane':
                    right_centroid = (cx, cy)
                    cv2.circle(annotated_frame, right_centroid, 7, (0, 255, 255), -1)
                    cv2.putText(annotated_frame, "RIGHT LANE", (cx - 20, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # [B] 특수 노면 표식 인식 (횡단보도, 직진 중 측면 빠지는 선)
            elif cls_name in ['crosswalk', 'cross_lane']:
                M = cv2.moments(pts)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(annotated_frame, (cx, cy), 8, (255, 0, 255), -1)
                    
                    if cls_name == 'crosswalk':
                        cv2.putText(annotated_frame, "WARNING: CROSSWALK", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                    elif cls_name == 'cross_lane':
                        cv2.putText(annotated_frame, "EVENT: CROSS LANE", (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    # 5. Target Point 계산
    target_x, target_y = None, None

    if left_centroid and right_centroid:
        # 양쪽 차선이 모두 보일 때 (정상 주행)
        cv2.line(annotated_frame, left_centroid, right_centroid, (255, 0, 0), 2)
        target_x = int((left_centroid[0] + right_centroid[0]) / 2)
        target_y = int((left_centroid[1] + right_centroid[1]) / 2)

    elif left_centroid:
        # 왼쪽 차선만 보일 때 (우측 차선 이탈 방지용 가상 타겟 설정)
        target_x = left_centroid[0] + LANE_WIDTH_HALF
        target_y = left_centroid[1]
        cv2.line(annotated_frame, left_centroid, (target_x, target_y), (0, 165, 255), 2)

    elif right_centroid:
        # 오른쪽 차선만 보일 때 (좌측 차선 이탈 방지용 가상 타겟 설정)
        target_x = right_centroid[0] - LANE_WIDTH_HALF
        target_y = right_centroid[1]
        cv2.line(annotated_frame, right_centroid, (target_x, target_y), (0, 165, 255), 2)

    # 6. Target Point 시각화
    if target_x is not None and target_y is not None:
        target_point = (target_x, target_y)
        cv2.circle(annotated_frame, target_point, 9, (0, 0, 255), -1)
        cv2.drawMarker(annotated_frame, target_point, (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)

        text = f"Target: ({target_x}, {target_y})"
        cv2.putText(annotated_frame, text, (target_x - 60, target_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
    out.write(annotated_frame)
    cv2.imshow("Robot Vision: 4-Class Seg & Targeting", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
print(f"완료! 저장 위치: {OUTPUT_VIDEO}")