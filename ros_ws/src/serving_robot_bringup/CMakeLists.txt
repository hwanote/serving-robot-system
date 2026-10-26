cmake_minimum_required(VERSION 3.8)

project(serving_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

# Shared team configuration (single source of truth lives in <repo>/config/common)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common
  DESTINATION share/${PROJECT_NAME}/config
)

# Helper used by operation_mode.launch.py to wait for readiness conditions
install(
  PROGRAMS scripts/wait_for_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
