# fleet_mission -> fleet_coordinator : 실행 가능한 작업 단위 (조정 층 입력)
string task_id
string mission_id
string type                  # "NAV_GOTO" | "CANCEL"
string robot_id              # 비어 있으면 조정 층이 배정
geometry_msgs/PoseStamped goal
