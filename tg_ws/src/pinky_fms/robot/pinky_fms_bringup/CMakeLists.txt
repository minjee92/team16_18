cmake_minimum_required(VERSION 3.8)
project(pinky_fms_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/robot_scan_filter.py scripts/fms_lcd_status.py scripts/fms_lane_mission.py scripts/fms_escape.py scripts/fms_camera_stream.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
