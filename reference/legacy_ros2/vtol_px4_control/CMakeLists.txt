cmake_minimum_required(VERSION 3.8)
project(vtol_px4_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(
  vtol_state_monitor
  src/vtol_state_monitor.cpp
)

ament_target_dependencies(
  vtol_state_monitor
  rclcpp
  px4_msgs
)

add_executable(
  vtol_command_node
  src/vtol_command_node.cpp
)

ament_target_dependencies(
  vtol_command_node
  geometry_msgs
  rclcpp
  px4_msgs
  std_msgs
  std_srvs
)

install(
  TARGETS
    vtol_state_monitor
    vtol_command_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
