# Pinky FMS (다중 로봇 관제 플랫폼)

Pinky Pro 로봇 여러 대를 웹 GUI 로 관제한다. 기존 `pinky_pro` 저장소는 **수정하지 않고**, 관제용 코드만 이 저장소에 둔다.
모든 로봇은 **같은 ROS_DOMAIN_ID(16)** 안에서 namespace(`amr_01`, `amr_02` ...)로 구분한다. 로봇 ID·IP·비밀번호는 코드에 쓰지 않고 GUI 에서 입력한다.

## 어디서 실행하는 파일인가

| 폴더 | 실행하는 곳 | 내용 |
|---|---|---|
| `control_pc/` | **관제 PC** | `ros/` ROS 2 패키지(`pinky_fms_interfaces` 메시지, `pinky_fms_core` 미션·조정 층, `pinky_fms_traffic` 충돌·교착 방지), `backend/` FastAPI(로봇 ON/OFF·SSH·로그·지도·ping), `web/` 웹 GUI, `fms_env.sh`(DDS·도메인 설정) |
| `robot/` | **로봇(RPi)** | `pinky_fms_bringup` — 기존 bringup/Nav2 launch 를 namespace 로 감싼 래퍼, 공용 Nav2 파라미터, 라이다 필터 |
| `simulation/` | 관제 PC (Gazebo) | `pinky_fms_sim` — 실물 지도로 만든 월드와 로봇 launch. **로봇에는 필요 없다** |
| `scripts/` | 관제 PC | `run_real.sh`(실물 스택 한 번에), `run_sim.sh`(가제보+관제), `run_demo.sh`(가짜 로봇) |
| `maps/` | 공용 | 사용하는 지도(`mission4_3_clean_1cm` 등). 로봇 홈에도 복사해서 Nav2 에 넘긴다 |
| `tools/` | 관제 PC | 미션·코스트맵 기록기, 결과 요약, 도착 허용 오차 분석 |
| `docs/` | | 상세 설명(`details.md`), 네트워크 점검표 |

```
웹 GUI ─ HTTP ─► backend ─ SSH ─► 로봇: robot_bringup(+ robot_nav: Nav2·AMCL·라이다 필터)
   │
   └─ rosbridge(9090) ─► fleet_mission ─► fleet_traffic ─► /amr_01/navigate_to_pose (실행 층 = 로봇의 Nav2)
```
층 규칙: 위층은 목표만 주고 아래층은 결과·상태만 올린다. 충돌·교착 방지(`fleet_traffic`)는 `fleet_coordinator` 를 상속한 조정 층이다.

## 빌드

저장소를 colcon 작업공간의 `src/` 아래에 둔다 (예: `~/pinky/src/pinky_fms`). colcon 이 하위 폴더를 재귀로 찾는다.

```bash
# 관제 PC
cd ~/pinky && source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_fms_interfaces pinky_fms_core pinky_fms_traffic pinky_fms_bringup pinky_fms_sim
# 백엔드 (한 번)
cd src/pinky_fms/control_pc/backend && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# 로봇 (각 로봇에서. 저장소 전체를 받아도 아래처럼 이 패키지만 빌드한다 — 나머지는 관제 PC/시뮬용이라 로봇에 의존성이 없다)
cd ~/pinky_pro && source /opt/ros/jazzy/setup.bash && colcon build --packages-select pinky_fms_bringup
```
`pinky_fms_sim` 은 `pinky_description`, `pinky_gz_sim`(pinky_pro) 과 `ros_gz` 가 필요하다.

## 실행 (터미널별)

관제 PC 의 모든 터미널에서 먼저:
```bash
source /opt/ros/jazzy/setup.bash && source ~/pinky/install/setup.bash
source <저장소>/control_pc/fms_env.sh       # domain, DDS(unicast) 설정
```
| 터미널 | 위치 | 명령 |
|---|---|---|
| 1 rosbridge | 관제 PC | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| 2 관제 노드 | 관제 PC | `ros2 launch pinky_fms_traffic traffic_core.launch.xml robots_file:=<저장소>/control_pc/ros/pinky_fms_core/config/robots.yaml map_yaml:=<저장소>/maps/mission4_3_clean_1cm.yaml` |
| 3 백엔드 | 관제 PC | `cd <저장소>/control_pc/backend && .venv/bin/python -m uvicorn app:app --host 127.0.0.1 --port 8000` |
| 4 웹 | 관제 PC | `cd <저장소>/control_pc/web && python3 -m http.server 8080` → http://localhost:8080 |
| A bringup | 로봇 | GUI 의 **ON** 이 SSH 로 자동 실행 (`ros2 launch pinky_fms_bringup robot_bringup.launch.xml namespace:=amr_01`) |
| B Nav2 | 로봇 | `ros2 launch pinky_fms_bringup robot_nav.launch.xml namespace:=amr_01 map:=$HOME/mission4_3_clean_1cm.yaml` (라이다 필터 포함) |

1~4 를 한 번에: `scripts/run_real.sh`. 가제보: `scripts/run_sim.sh`. 가짜 로봇: `scripts/run_demo.sh`.
관제 노드(2)는 기존 `fms_core.launch.xml` 대신이다 (같이 띄우면 안 된다).

## 로봇에 파일 가져오기

```bash
git clone <저장소 주소> ~/pinky_pro/src/pinky_fms     # 또는 scp
scp <저장소>/maps/mission4_3_clean_1cm.* pinky@<로봇 IP>:~/   # 지도는 홈에 둔다
```

## 시험

- 충돌 방지 알고리즘(ROS 없이): `python3 control_pc/ros/pinky_fms_traffic/tests/scenarios.py [map.yaml] [랜덤 횟수]`
- ROS 격리 시험(도메인 91, 가짜 로봇): `control_pc/ros/pinky_fms_traffic/tests/run_ros_test.sh <headon|follow|parked|deadlock|three> [traffic|plain]`
- 자세한 설계: `control_pc/ros/pinky_fms_traffic/README.md`, 등록·네트워크: `docs/details.md`
