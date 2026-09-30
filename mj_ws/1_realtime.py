import torch, torchvision
import cv2
from ultralytics import YOLO

CAM =0
WHEIGHTS = 'pouch_best.pt'

model = YOLO(WHEIGHTS)
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