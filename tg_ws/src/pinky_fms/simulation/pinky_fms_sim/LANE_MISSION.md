# Gazebo 에서 실물 차선 미션 노드 돌리기 (한 대)

관제 `lane_route` → 실물 `fms_lane_mission`(가짜 Picamera2 = Gazebo 카메라) 이 목표까지 차선을 따라간다.
2026-10-08 확인: R1 → 꼬리 윗길 (0.45, 0.95) 1.62 m 도착, 이어서 J 직진 → 고리 오른변 (2.0, 0.80) 1.72 m 도착.
Gazebo 에서는 YOLO 가 cross_lane 을 못 잡았지만 갈림길 동작은 위치로 시작해 문제없었다.

준비 (한 번): ultralytics 가상환경 (`--system-site-packages`)
```bash
python3 -m venv --system-site-packages ~/dev_ws/yolo_sim_venv
~/dev_ws/yolo_sim_venv/bin/pip install torch==2.14.1+cpu torchvision==0.29.1+cpu --index-url https://download.pytorch.org/whl/cpu
~/dev_ws/yolo_sim_venv/bin/pip install -r <tg_ws>/src/pinky_fms/simulation/pinky_fms_sim/requirements-lane.txt
```

모든 터미널에서 먼저:
```bash
source /opt/ros/jazzy/setup.bash && source ~/pinky/install/setup.bash && source ~/dev_ws/team16_18/tg_ws/install/setup.bash
export ROS_DOMAIN_ID=93 GZ_PARTITION=tg_lane RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI
R=~/dev_ws/team16_18/tg_ws/src/pinky_fms; B=$R/control_pc/backend/maps
```

| 터미널 | 명령 |
|---|---|
| 1 월드 (화면) | `ros2 launch pinky_fms_sim sim_world.launch.xml` |
| 2 로봇 (R1) | `ros2 launch pinky_fms_sim sim_robot.launch.xml namespace:=amr_01 x:=0.783 y:=0.253 yaw:=3.14159 nav:=false` |
| 3 미션 노드 | `ros2 launch pinky_fms_sim sim_lane_mission.launch.xml namespace:=amr_01 model_path:=$HOME/dev_ws/yolo_mission/lane_model_ncnn/best.pt python:=$HOME/dev_ws/yolo_sim_venv/bin/python` → `목적지 대기 중` |
| 4 관제 | `ros2 run pinky_fms_traffic lane_traffic --ros-args -p lanes_yaml:=$B/mission4_3_lanes_1cm/map.yaml -p floor_yaml:=$B/mission4_3_nolanes_1cm/map.yaml -p "escape_block_rects:=-0.14,-0.07,-0.12,0.17" & ros2 launch pinky_fms_lane lane_route.launch.xml map_yaml:=$R/maps/mission4_3_clean_1cm.yaml` |
| 5 초기 위치 | `ros2 run pinky_fms_lane send_initial_pose amr_01:R1 --map $R/maps/mission4_3_clean_1cm.yaml --frame-prefix amr_01/ --no-image` → `→ OK` |
| 6 목표 | `ros2 topic pub -t 3 -r 1 /fleet/lane_goal std_msgs/msg/String "{data: '{\"robot\":\"amr_01\",\"id\":1,\"cmd\":\"home\",\"x\":0.45,\"y\":0.95}'}"` (새 목표마다 id 를 바꾼다. go = 왕복) |

보기: `ros2 topic echo /amr_01/lane_status` (state, route.s/s_goal), 터미널 3 로그 (`🗺️ 관제 경로`, `🔀 갈림길 J 구역 진입`, `🏁 목적지 도착`).

한계·주의:
- 바닥 그림이 줄자 코스와 꼬리 아랫길 −6 cm, 고리 아랫변 +5 cm (폭 11 cm) 어긋난다 (나머지 ±2 cm). 아랫변·꼬리 아랫길에서 경로 벗어남 경고가 날 수 있다.
- 초음파·LED·LCD 없음 (앞 사물 정지는 라이다만). 두 대 마주침은 로봇·미션 노드를 amr_02 로 하나 더 띄우면 되지만 CPU 부담이 커 아직 안 해 봤다.
- 화면 없는 실행(`headless_rendering:=true`)은 이 노트북에서 Gazebo 가 죽었다 (렌더링). 화면 있는 기본값을 쓴다.
