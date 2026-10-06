# fleet_coordinator -> fleet_mission -> GUI : 작업 진행 상태 (결과/상태만 위로 올린다)
string task_id
string mission_id
string robot_id              # 배정된 로봇
string state                 # PENDING | ASSIGNED | RUNNING | SUCCEEDED | FAILED | CANCELED
float32 distance_remaining   # m, RUNNING 중에만 유효
string message
