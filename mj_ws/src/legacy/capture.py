# legacy: 초기 학습 데이터 수집용, 현재 데이터셋은 Roboflow로 관리
import os
import cv2
from datetime import datetime
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent.parent   # mj_ws/
SAVE_DIR = str(BASE_DIR / 'tmp')
CAMERA_ID = 0

def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print(f'카메라를 열 수 없습니다 CAMERA_ID = {CAMERA_ID}')
        return


    print('=' * 50)
    print(' Enter : 사진 저장')
    print(' q     : 종료')
    print('=' * 50)

    count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print('프레임을 읽지 못했습니다')
            break


        preview = frame.copy()
        cv2.putText(
            preview, f'Saved: {count}  [Enter] save  [q] quit',
            (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, ( 0,255,0), 2
        )

        cv2.imshow('Webcam', preview)

        key = cv2.waitKey(1) & 0xFF

        if key == 13 or key == 10:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = os.path.join(SAVE_DIR, f'img_{timestamp}.jpg')

            cv2.imwrite(filename, frame)

            count +=1
            print(f'[{count}] 저장됨 : {filename}')

        elif key == ord('q'):
            break



    cap.release()
    cv2.destroyAllWindows()
    print(f'\n총 {count}장 저장했습니다. 폴더: {os.path.abspath(SAVE_DIR)}')


if __name__ == '__main__':
    print('main 시작')
    main()
