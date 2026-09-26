cmake_minimum_required(VERSION 3.8)
project(minicar_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/vehicle_sim.py
  scripts/sim_viz.py
  scripts/course.py
  scripts/camera_info_pub.py
  scripts/export_unity_course.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
