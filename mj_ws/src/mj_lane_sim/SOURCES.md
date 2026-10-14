# mj_lane_sim 출처

원본: 팀 저장소 `origin/feat/tg-gazebo-course` 커밋 **`fb2f983`** 의 `tg_ws/src/pinky_fms/simulation/pinky_fms_sim` (GPT 작성).
`git show fb2f983:<경로>` 로 내용만 가져왔다 (이력은 가져오지 않음). 텍스트 파일은 머리 주석에도 원본 경로를 적었다.

## 가져온 파일

| mj_lane_sim | 원본 (fb2f983) | blob | 변경 |
|---|---|---|---|
| `worlds/mission4_3_textured.sdf` | `pinky_fms_sim/worlds/mission4_3_textured.sdf` | `6b02c06e701a` | 머리 주석만 |
| `worlds/mission4_3_textured_assets/course.png` | 같은 이름 | `ea694f296400` | 없음 (바이트 동일) |
| `worlds/mission4_3_textured_assets/ground.dae` | 같은 이름 | `14c73dd6bc91` | 없음 |
| `worlds/mission4_3_textured_assets/calibration.json` | 같은 이름 | `1d6fb55b3352` | 없음 |
| `config/course_texture.json` | `pinky_fms_sim/config/course_texture.json` | `0d3289c7f090` | 없음 (참고용 보정 기록) |
| `maps/mission4_3_clean_1cm.pgm` | `tg_ws/src/pinky_fms/maps/mission4_3_clean_1cm.pgm` | `64ee931eabb5` | 없음 |
| `maps/mission4_3_clean_1cm.yaml` | `tg_ws/src/pinky_fms/maps/mission4_3_clean_1cm.yaml` | `fa861d0e0ae5` | 머리 주석만 |
| `scripts/smoke_sim.py` | `pinky_fms_sim/scripts/smoke_sim.py` | — | 머리 주석만 |
| `scripts/gz_remove_model.sh` | `pinky_fms_sim/scripts/gz_remove_model.sh` | — | 머리 주석만 |
| `launch/sim_world.launch.xml` | 같은 이름 | — | 패키지 이름 |
| `launch/sim_robot.launch.xml` | 같은 이름 | — | Nav2(`pinky_fms_bringup`)·`params_file`·`map`·가짜 배터리 제거, 카메라 기본값을 실물 값으로, 전방 초음파 흉내 노드 추가 |
| `scripts/sim_urdf.sh` | 같은 이름 | — | 화각·렌즈 높이 인자 추가, 기본 해상도 640×480 |
| `scripts/sim_camera.py` | 같은 이름 | — | 화각(`--hfov-deg`)·렌즈 높이(`--lens-height`) 바꾸기 추가 |

## mj 가 새로 만든 파일 (원본 없음)
| 파일 | 하는 일 |
|---|---|
| `scripts/sim_front_range.py` | 라이다 → `/<ns>/us_sensor/range` (실물 초음파 드라이버와 같은 Range 형식, 상한 0.97 m). 시뮬에 초음파가 없어서 |
| `scripts/run_mj_sim.sh` | `world / robot / camera / teleop / drive / stop`. 도메인 95·`GZ_PARTITION=mj_sim`·LOCALHOST 로만 띄운다 |

## 가져오지 않은 것과 이유
- `sim_lane_follow.py`, `sim_mission.py`, `tests/check_autonomous_states.py`: 팀원 `pinky_autonomous` 차선 코드에 의존. 판단 코드는 mj 것 하나만 쓴다 (`mj_lane_drive`).
- `fake_battery.sh`, `config/robots.sim.yaml`: 팀 관제(FMS)용.
- `worlds/mission4_3_clean_1cm.sdf` (바닥 그림 없는 옛 월드), `worlds/textures/course_gray_white_lanes.png` (`course.png` 와 같은 파일).
- `tools/*`, `tests/test_ground_texture.py`, `docs/validation/` (약 6 MB), `ANALYSIS.md`, `TEXTURE.md`, `requirements-lane.txt`: 바닥 그림을 다시 만들 때만 필요. 원본 브랜치에 남아 있다.

## 바닥 그림 보정과 한계 (원본 TEXTURE.md 요약 + mj A1 측정)
- 바닥 그림(1711×919 px)은 `clean_1cm` 지도의 벽 픽셀에 맞춰 단조 격자로 붙였다 (`calibration.json`, method `monotone_grid`).
  원본 문서: 실제 테이프 폭이나 카메라 외부 파라미터를 잰 것은 아니다.
- `clean_1cm` 지도는 실물보다 세로 7–8 cm, 가로 3–5 cm 작다 (SLAM 원본이 약 2° 돌아간 채 정리됨). 줄자: 아레나 233×128 cm.
- mj A1 비교 (2026-10-12, 줄자 `mission4_3_clean_1cm.tape.yaml` 과): 위쪽 길은 ±0.6 cm 로 맞다.
  **고리 아랫변 폭 11.2 cm (줄자 16.6 cm), 꼬리 아랫길 6.5 cm 벽 쪽으로 밀림**, 꼬리 왼쪽 길은 2.5–3.4 cm 넓음.
  → 아래쪽 구간의 시뮬 결과는 "실물과 다름". 바로잡기는 PLAN 의 "지도 바로잡기" 단계에서 한다.

## 로봇 모델
패키지 안에 없다. `~/pinky` 의 `pinky_description`·`pinky_gz_sim` (pinky_pro `75f76e8`) 을 쓴다.
원본 문서가 가정한 버전은 `014a09f` 로 다르지만, mj A1 에서 원본 시뮬 smoke 시험이 이 버전으로 통과했다.
