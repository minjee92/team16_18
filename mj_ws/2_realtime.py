import cv2
from ultralytics import YOLO

INPUT_SOURCE = '/home/mindy/dev_ws/yolo_mission/pinky_20260919_182009.mp4'

MODEL_PATH = 'best.pt'
OUTPUT_VIDEO = '/home/mindy/dev_ws/yolo_mission/result.mp4'

# 1. 모델 로드
model = YOLO(MODEL_PATH)

# 2. 비디오 캡처 (INPUT_SOURCE가 경로면 동영상 파일, 0이면 웹캠으로 작동)
cap = cv2.VideoCapture(INPUT_SOURCE)
if not cap.isOpened():
    print(f"오류: 입력 소스를 열 수 없습니다 -> {INPUT_SOURCE}")
    exit()

# 3. 해상도 및 FPS 가져오기
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps    = cap.get(cv2.CAP_PROP_FPS)
fps    = fps if fps > 0 else 30.0  # 웹캠일 경우 FPS 값이 0으로 들어오는 경우 방지

# 4. 저장 설정
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (width, height))

print("추론 시작... 종료하려면 'q'를 누르세요.")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 추론 및 시각화
    results = model.predict(source=frame, conf=0.25, verbose=False)
    annotated_frame = results[0].plot()

    # 저장 및 화면 출력
    out.write(annotated_frame)
    cv2.imshow("YOLO11 Segmentation", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
print(f"완료! 저장 위치: {OUTPUT_VIDEO}")

