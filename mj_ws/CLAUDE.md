# yolo_mission — 차선 세그멘테이션 프로젝트

자율주행 학습 MVP. 실내 트랙(회색 카펫 + 흰색 테이프)에서 촬영한 영상으로
차선 세그멘테이션 모델을 학습한다.

## 데이터셋

`lane_seg.v5i.coco-segmentation/` — Roboflow COCO Segmentation 포맷

- 842장 (train 589 / valid 160 / test 93), 전부 640×640
- 어노테이션 1,405개
- 전처리: Auto-orient + Resize 640×640 (Fit, black edges) → 위아래 검은 띠 있음
- 증강: 밝기 + 상하 이동 (좌우반전 증강은 **없음**)

### 클래스

| id | 이름 | 설명 |
|---|---|---|
| 0 | lane-seg | 더미 루트 (사용 안 함) |
| 2 | cross_lane | 교차로에서 가로지르는 차선 |
| 3 | crosswalk | 횡단보도 |
| 4 | left_lane | 로봇 기준 왼쪽 차선 |
| 5 | right_lane | 로봇 기준 오른쪽 차선 |

**left/right 판별 기준은 화면 위치가 아니라 프레임 하단에서 로봇 기준 좌·우다.**
선이 화면 중앙~오른쪽에 보여도, 로봇이 그 선의 오른쪽을 주행하면 `left_lane`이다.

`categories` 목록의 id 1(이름 `"2"`)은 Roboflow가 만든 오류 클래스이며
어노테이션은 모두 제거했다. **목록 자체는 지우지 말 것** — id 매핑이 깨진다.

## 중요: 좌우반전 이력

원본 데이터는 좌우반전된 영상으로 라벨링되어 있었다. 2026-09 기준으로
이미지·좌표·클래스를 모두 반전 처리해 **정방향으로 교정 완료**했다.

- 이미지: `cv2.flip(img, 1)`
- 폴리곤: `x' = W - x`, 점 순서 역순 (감김 방향 보존)
- bbox: `x' = W - (x + w)`, y·w·h·area 불변
- 클래스: annotation의 `category_id` 4↔5 교체 (categories 목록은 원본 유지)

**`flip_dataset.py` / `flip_images.py`를 다시 실행하지 말 것.** 한 번 더 뒤집혀
원상복구된다. 각 split 폴더의 `.images_flipped` 표식이 처리 완료를 뜻한다.
상태 확인은 `python3 flip_images.py . --check`.

json이 반전된 상태인지는 `info.description`에 `horizontally flipped`
문자열이 있는지로 판별한다.

## 알려진 이슈 (미해결)

**split 누수** — 같은 원본 프레임에서 증강된 이미지들이 train/valid/test에
흩어져 있다. 65개 프레임이 2개 이상 split에 걸쳐 있다. 이대로 학습하면
valid/test 점수가 실제 성능보다 높게 나온다. 재분할이 필요하다.

## 스크립트

- `flip_dataset.py` — 원본 → 반전본 전체 변환 (이미지 + json + 클래스)
- `flip_images.py` — 이미지만 반전, json 상태 자동 판별
- `auto_label_lanes.py` — 흰색 테이프 자동 검출 → COCO 폴리곤 생성.
  기존 GT 대비 IoU 중앙값 0.87. `crosswalk` / `cross_lane`은 자동 배정하지
  않으므로 사람이 지정해야 한다.

## 작업 규칙

- 데이터셋 파일을 제자리에서 수정하기 전에 반드시 백업하거나 새 폴더로 출력할 것
- 어노테이션을 변경했으면 오버레이 이미지를 렌더링해 눈으로 검증할 것
  (파란색 left_lane이 왼쪽, 주황색 right_lane이 오른쪽에 붙어야 정상)
- 좌표 변환 후에는 왕복 복원 테스트로 오차가 0인지 확인할 것
