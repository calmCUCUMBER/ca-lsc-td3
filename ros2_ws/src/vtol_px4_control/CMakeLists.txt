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
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(
  ca_lsc_instrumented_lift_drag SHARED
  src/instrumented_lift_drag.cpp
)

target_link_libraries(
  ca_lsc_instrumented_lift_drag
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

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
  gz_wind_bridge
  src/gz_wind_bridge.cpp
)

ament_target_dependencies(
  gz_wind_bridge
  geometry_msgs
  rclcpp
)

target_link_libraries(
  gz_wind_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

add_executable(
  gz_wing_force_bridge
  src/gz_wing_force_bridge.cpp
)

ament_target_dependencies(
  gz_wing_force_bridge
  geometry_msgs
  rclcpp
  std_msgs
)

target_link_libraries(
  gz_wing_force_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
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
    gz_wind_bridge
    gz_wing_force_bridge
    ca_lsc_instrumented_lift_drag
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
