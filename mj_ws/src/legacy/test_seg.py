import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent.parent   # mj_ws/

# =========================================================================
# [이 부분 1개만 변경하세요]
# 1) 저장된 영상 추론: str(BASE_DIR / 'inputs' / 'my_video.mp4') (파일 경로 문자열)
# 2) 실시간 웹캠 추론: 0 (숫자)
INPUT_SOURCE = str(BASE_DIR / 'inputs' / 'pinky_20260919_182009.mp4')
# =========================================================================

MODEL_PATH = str(BASE_DIR / 'models' / '260928_yolon_best.pt')
OUTPUT_VIDEO = str(BASE_DIR / 'outputs' / 'result_test_seg.mp4')

# 1. 모델 로드 및 클래스 이름 매핑
model = YOLO(MODEL_PATH)
class_names = model.names # {0: 'left', 1: 'right'} 형태의 딕셔너리

# 2. 비디오 캡처 설정
cap = cv2.VideoCapture(INPUT_SOURCE)
if not cap.isOpened():
    print(f"오류: 입력 소스를 열 수 없습니다 -> {INPUT_SOURCE}")
    exit()

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
fps = fps if fps > 0 else 30.0

# 3. 비디오 저장 객체
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (width, height))

print("추론 및 Target Point 계산 시작... (종료: 'q')")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 4. 추론 실행
    results = model.predict(source=frame, conf=0.25, verbose=False)
    result = results[0]

    # 기본 YOLO 세그멘테이션 결과 오버레이 이미지 가져오기
    annotated_frame = result.plot()

    left_centroid = None
    right_centroid = None

    # 5. 마스크 데이터가 존재할 경우 중심점 계산
    if result.masks is not None and result.boxes is not None:
        masks_xy = result.masks.xy # 각 객체의 Polygon (x, y) 좌표 배열
        classes = result.boxes.cls.cpu().numpy() # 각 감지 객체의 클래스 ID List

        for mask_pts, cls_id in zip(masks_xy, classes):
            cls_name = class_names[int(cls_id)] # 클래스 이름 ('left' 또는 'right')

            if len(mask_pts) == 0:
                continue

            # 다각형 좌표를 OpenCV 형식(int32)으로 변환
            pts = np.array(mask_pts, dtype=np.int32)

            # Moments를 이용한 다각형의 기하학적 중심점(Centroid) 계산
            M = cv2.moments(pts)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                if cls_name == 'left_lane':
                    left_centroid = (cx, cy)
                elif cls_name == 'right':
                    right_centroid = (cx, cy)

    # 6. 각 차선의 중심점 시각화 (노란색 원)
    if left_centroid:
        cv2.circle(annotated_frame, left_centroid, 7, (0, 255, 255), -1)
        cv2.putText(annotated_frame, "Left", (left_centroid[0] - 20, left_centroid[1] - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    if right_centroid:
        cv2.circle(annotated_frame, right_centroid, 7, (0, 255, 255), -1)
        cv2.putText(annotated_frame, "Right", (right_centroid[0] - 20, right_centroid[1] - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # 7. 양쪽 차선이 모두 감지되었을 때 Target Point 계산 및 시각화
    if left_centroid and right_centroid:
        # 두 중심점 사이의 연결선 그리기 (파란색 선)

        # 중앙 target_point 계산
        target_x = int((left_centroid[0] + right_centroid[0]) / 2)
        target_y = int((left_centroid[1] + right_centroid[1]) / 2)
        target_point = (target_x, target_y)

        # Target Point 시각화 (빨간색 원 및 십자가 표시)
        cv2.circle(annotated_frame, target_point, 9, (0, 0, 255), -1)
        cv2.drawMarker(annotated_frame, target_point, (255, 255, 255),markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)

        # 텍스트 정보 표시
        text = f"Target: ({target_x}, {target_y})"
        cv2.putText(annotated_frame, text, (target_x - 60, target_y - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # [참고] 향후 로봇으로 전송할 데이터 구조 예시
        # robot_control_data = {"target_x": target_x, "target_y": target_y}

    # 8. 저장 및 출력
    out.write(annotated_frame)
    cv2.imshow("Lane Tracking - Robot Target Point", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
print(f"완료! 저장 위치: {OUTPUT_VIDEO}")