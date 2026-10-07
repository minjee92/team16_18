cmake_minimum_required(VERSION 3.8)
project(pinky_fms_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/robot_scan_filter.py scripts/fms_lcd_status.py scripts/fms_lane_mission.py scripts/fms_escape.py
  DESTINATION lib/${PROJECT_NAME}
)
# fms_lane_mission 이 import 하는 모듈 (실행 파일이 아님, 같은 폴더에 둔다)
install(FILES scripts/lane_route_track.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
