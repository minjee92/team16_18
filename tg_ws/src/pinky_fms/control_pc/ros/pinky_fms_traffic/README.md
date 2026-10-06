# pinky_fms_traffic — 다중 로봇 충돌·교착 방지 (1단계: ROS 없이 순수 Python)

기존 fms 패키지(`fleet_mission`, `fleet_coordinator`, GUI, backend)는 **건드리지 않는다**. `package.xml` 이 없으므로 colcon 빌드에도 잡히지 않는다.

## 방식 (고정 대기 장소 없음)
매 주기마다 로봇들의 **남은 계획 경로**를 서로 겹쳐 본다.
1. 두 경로가 `R_CONF`(0.24 m) 이내로 가까워지고 도착 시각이 비슷하면(시간 간격 + ETA 불확실도) 충돌 예정.
2. ETA 가 작은 쪽이 우선 (서 있는 로봇은 항상 후순위, 한 번 정해진 우선권은 충돌이 사라질 때까지 유지).
3. 진 쪽: 충돌 구간 앞에서 HOLD (좁은 통로면 입구 앞) / 이미 상대 경로 위면 YIELD (상대 남은 경로·예상 위치에서 벗어난 가장 가까운 칸으로).
4. 비킬 곳이 없으면 역할을 바꿔 본다. 그래도 안 되면 둘 다 정지(교착 → 사람에게 알려야 하는 경우).
5. 상태를 따로 저장하지 않고 매 주기 재계산하므로, 상대가 지나가면 자동으로 풀린다.

통로는 지도에서 자동 분류한다 (좌표를 코드에 쓰지 않는다 → 맵이 바뀌어도 동작): 가운데 선(여유 국소 최대)의 여유×2 = 통로 폭.
- 폭 < 0.42 m : 1차선 (두 대가 나란히 설 수 없음) → 진입 전 입구 앞(완충 0.24 m 밖)에서 대기, 이미 안이면 통로 밖으로 더 짧게 되돌아 나가는 쪽이 물러난다
- 0.42~0.64 m : 한계 폭 (오른쪽 차선 규칙 필요, 미구현)
- 0.64 m 이상 : 2차선 이상
비켜설 자리: 상대 경로·예상 위치에서 0.22 m 이상, 벽 여유 0.10 m 이상, 1차선 입구 완충 밖.

## 실행
    python3 tests/scenarios.py [map.yaml] [랜덤 횟수]
## 구성
- `gridmap.py` 지도 → 격자·여유 거리, `planner.py` A*(벽 근처 비용), `traffic.py` 판단 로직, `sim.py` 시뮬레이터
## 한계 (1단계)
- 시뮬레이터는 질점+원형 몸체(중심 거리 < 0.19 m = 충돌), 속도 0.15~0.2 m/s 무작위. 실제 Nav2 의 감속·재계획·AMCL 오차는 반영 안 됨.
- 2대 기준으로 검증. 3대 이상은 쌍별 규칙이라 동작은 하나 시험하지 않았다.
- 목표 지점이 서로 가깝거나(0.35 m 미만) 한쪽 시작점이 다른 쪽 목표인 경우는 풀 수 없어 시험에서 제외.

## 2단계: ROS 노드 (fleet_traffic)
`fleet_traffic` 은 `fleet_coordinator` 를 상속한다 (기존 코드 수정 없음). **fleet_coordinator 대신** 띄운다.

    # 실제/가짜 로봇 환경에서 (fms_core.launch.xml 대신)
    ros2 launch pinky_fms_traffic traffic_core.launch.xml robots_file:=<robots.yaml> map_yaml:=<지도.yaml>
    # map_yaml 을 비우면 충돌 방지 꺼짐 = fleet_coordinator 와 동일

- 로봇별 `/<ns>/compute_path_to_pose`(Nav2 계획 경로, 없으면 내부 A*)를 받아 TrafficManager 에 넘긴다.
- 로봇에는 NavigateToPose 목표 하나만 보낸다: 최종 목표(GO) / 충돌 구간 앞 대기 지점(HOLD) / 비켜설 자리(YIELD, passing bay).
- GUI 에는 원래 미션 ID 로 RUNNING(+메시지 "충돌 방지: …")이 올라가고, 최종 목표 도착 때만 SUCCEEDED.
- 서 있는 로봇(미션 없음)이 길을 막으면 미션 없이 비켜서고, 그 동안은 BUSY 로 보여 새 미션을 받지 않는다.
- 격리 시험(도메인 91, 이 PC 안에서만, 실제 스택 무관): `tests/run_ros_test.sh <headon|follow|parked> [traffic|plain]`
  (지도를 아는 가짜 로봇 `traffic_mock_robot` 사용)

### 실제 Nav2 에 붙일 때 확인할 것 (가짜 로봇으로는 검증 못 함)
- bt_navigator 가 NavigateToPose 새 목표를 받으면 이전 목표를 대체(preempt)하는지
- 대기 지점 도착 허용 오차(xy 0.05 m)와 AMCL 오차(±0.1 m 이상) — 대기·비켜서기 여유(R_CONF 0.24 m)에 반영돼 있는지
- 1차선 통로 안에서 되돌아 나가기는 후진이 필요 (allow_reversing=false). 입구 대기가 우선이고 되돌아 나가기는 보조
- 컨트롤러 progress_checker(10 s 안에 0.5 m 이동)는 목표를 보내 두고 서 있을 때만 걸리므로, 이 방식(대기 지점 목표)에서는 문제가 되지 않아야 함
