# pinky_mini_mission

Pinky 미니 미션 노드 모음 (ROS 2, ament_python). `pinky_mission` 패키지에서 분리했다.

| 실행 이름 | 내용 | 필요한 것 |
|---|---|---|
| `mission1` | 0.7 m 직진 → 180° 회전 → 출발점 복귀 (`/odom` 기준, `/cmd_vel` 발행) | 로봇 bringup |
| `mission2` | 고정 waypoint 2곳을 거쳐 출발 위치로 복귀 (`follow_waypoints` 액션) | Nav2 + 위치추정 |
| `mission3_1` | RViz 의 Publish Point(`/clicked_point`)로 목적지를 찍고 Enter → 순서대로 주행 후 복귀, 2초마다 현재 위치 출력 | Nav2 + 위치추정 |

```bash
colcon build --packages-select pinky_mini_mission
source install/setup.bash
ros2 run pinky_mini_mission mission1
```
