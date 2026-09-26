cmake_minimum_required(VERSION 3.8)
project(minicar_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneInfo.msg"
  "msg/ArrowSign.msg"
  "msg/SurfaceState.msg"
  "msg/ZoneState.msg"
  "msg/DriveCommand.msg"
  "msg/VehicleStatus.msg"
  "msg/RaceStatus.msg"
  "msg/ParkingTarget.msg"
  "msg/OpponentInfo.msg"
  "msg/FusionState.msg"
  "msg/PlanStatus.msg"
  "msg/ActuatorCmd.msg"
  "msg/LookAhead.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
