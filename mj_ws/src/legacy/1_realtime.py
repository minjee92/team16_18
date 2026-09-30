# legacy: 초기 파우치 검출 테스트용, 차선 세그멘테이션과 무관
import torch, torchvision
import cv2
from ultralytics import YOLO
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent.parent   # mj_ws/

CAM =0
MODEL_PATH = str(BASE_DIR / 'tmp' / 'legacy_models' / 'old_pt' / 'pouch_best.pt')

model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(CAM, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

while True:
    ok, frame = cap.read()
    if not ok:
        break

    r= model.predict(frame, conf=0.5, imgsz=640, verbose=False)[0]
    cv2.imshow('yolo', r.plot())
    if cv2.waitKey(1) & 0xFF ==27:
        break


cap.release()
cv2.destroyAllWindows()