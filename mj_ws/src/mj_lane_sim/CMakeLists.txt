cmake_minimum_required(VERSION 3.8)
project(mj_lane_sim)
# 원본 pinky_fms_sim(fb2f983) 의 CMakeLists 를 바탕으로 함. 지도는 패키지 안 maps/ 만 설치한다 (원본은 ../../maps)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_urdf.sh scripts/sim_camera.py scripts/gz_remove_model.sh scripts/smoke_sim.py
  scripts/sim_front_range.py scripts/run_mj_sim.sh
  DESTINATION lib/${PROJECT_NAME})
ament_package()
