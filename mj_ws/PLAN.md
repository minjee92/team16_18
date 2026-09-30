# 개발 계획 및 진행 상황

차선 인식 기반 자율주행. 저장 영상 검증부터 관제 GUI까지 4단계로 진행한다.

## 단계

| 단계 | 내용 | 모델 | 실행 위치 | 상태 |
|---|---|---|---|---|
| 1 | 저장 영상으로 left/right 차선 인식 확인 | `260928_yolon_best.pt` | PC | 완료 |
| 2 | 차선 가운데로 주행 | `lane_model_ncnn` | 로봇 | 대기 |
| 3 | 2번 + crosswalk 감지 시 3초 정지 후 재출발 | `lane_model_ncnn` | 로봇 | 대기 |
| 4 | 3번 + GUI로 지도에서 목표지점 클릭해 이동 | `lane_model_ncnn` | 로봇 + 관제 PC | 대기 |

4단계의 주행은 객체인식으로, 좌/우회전 판단은 좌표 계산으로 한다.

## 진행 이력

- **이관 완료** — `~/dev_ws/yolo_mission` → `mj_ws`, git 이력 관리 시작 (`aafea7e`)
- **정리 완료** — 폴더 구조 정리, 모델 역할 확정, legacy 분리 (`4e091a9`)
- **1단계 완료** — `src/lane_follow_check.py` 로 차선 인식 확인. 결과 영상 육안 검증 통과
- **진행 중** — 검증 기능 보강 + 후처리 분리 (`src/common/lane_postprocess.py`)

## 현재 작업: 후처리 분리

1단계와 2단계를 잇는 준비 작업이다.

`lane_follow_check.py` 안에서 "마스크를 받아 차선 중심·조향값을 계산하는" 부분을
`src/common/lane_postprocess.py` 로 떼어낸다. 이 계산은 모델 종류와 무관하므로,
2단계 ncnn 주행 코드에서 그대로 재사용한다.

검증 기준: **분리 전후 결과 영상이 픽셀 단위로 동일해야 한다.**
동작을 바꾸는 수정은 이 작업에 섞지 않는다.

함께 추가하는 것:

- 프레임별 클래스 검출 여부와 신뢰도 출력
- 요약 통계 (left/right/crosswalk 검출률, 양쪽 다 놓친 프레임 수)
- 입출력 경로 argparse

## 미뤄둔 것

| 항목 | 내용 | 언제 |
|---|---|---|
| 빨간 십자 버그 | `draw()` 가 목표점 마커를 `ROW_WEIGHTS[:len(centers)]` 로 다시 계산한다. 행이 비면 가중치가 밀려서, 화면에 그려지는 위치가 실제 조향에 쓰는 `target_x` 와 달라질 수 있다 | 후처리 분리 완료 후, 별도 커밋 |
| 후처리 복제본 | `lane_mission_drive.py` 에 같은 로직의 복사본이 있다. D항(`STEER_D_GAIN`), steer clip, `reset()` 이 추가돼 있어 검증 코드와 동작이 다르다 | 2단계에서 공통 모듈로 통합, 차이는 파라미터로 흡수 |
| 로봇 배포 경로 | 현재 모든 경로가 `BASE_DIR`(= `mj_ws/`) 기준이다. 로봇에는 이 폴더 구조가 없으므로 `robot/` 코드는 경로를 argparse 나 환경변수로 받아야 한다 | 2단계 설계 시 |

## 코드 배치 원칙

실행 위치로 나눈다. 로봇에 배포할 때 해당 폴더만 복사하면 되도록.

```
src/
├── common/     양쪽에서 쓰는 순수 계산 (차선 후처리, 좌표 변환)
│   └── lane_postprocess.py
├── robot/      로봇에서 실행. ncnn 추론 + 주행
│   ├── lane_mission_drive.py
│   └── model_ncnn.py
├── station/    관제 PC에서 실행. GUI
│   └── mission_gui.py
├── tools/      개발용. 저장 영상 검증 등
│   └── step1_check_lane_detection.py
└── legacy/     안 쓰지만 보관
    ├── 1_realtime.py
    ├── 2_realtime.py
    ├── capture.py
    ├── find_target_point.py
    ├── find_target_point_dual.py
    └── test_seg.py
```

이름 규칙:

- `step1_` ~ `step4_` 접두어는 위 4단계 계획의 결과물에만 붙인다
  (1단계 → `tools/step1_check_lane_detection.py`)
- 아직 기반 코드인 `robot/lane_mission_drive.py`, `station/mission_gui.py` 는 접두어 없이 이름을 유지한다

## 작업 규칙

- 한 단계가 **실제로 동작하는 것을 확인한 뒤** 다음 단계로 간다
- 리팩터링(동작 안 바뀜)과 기능 수정(동작 바뀜)을 같은 커밋에 섞지 않는다
- 단계마다 커밋해서 되돌릴 지점을 남긴다
- 저장소 밖 경로를 건드리는 작업은 Manual 모드로 진행한다
