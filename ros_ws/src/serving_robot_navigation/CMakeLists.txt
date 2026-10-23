cmake_minimum_required(VERSION 3.8)

project(serving_robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package serving_robot_navigation: shared site_config / interfaces, and the
# sections monitoring/ decision/ motion/ command/ (see docs/INTERFACES.md)
ament_python_install_package(${PROJECT_NAME})

# Command line tools (ros2 run serving_robot_navigation go_to.py ...)
install(
  PROGRAMS
    scripts/monitoring/dynamic_scan_filter.py
    scripts/monitoring/person_detector.py
    scripts/decision/person_state.py
    scripts/decision/robot_health.py
    scripts/motion/person_speed_limiter.py
    scripts/motion/soft_stop_limiter.py
    scripts/motion/hold_recovery.py
    scripts/command/route_executor.py
    scripts/command/go_to.py
    scripts/command/serve.py
    scripts/command/decide.py
    scripts/command/reset.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    behavior_trees
    config
    launch
    maps
    rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

# Static store maps live in the repository-level maps/stores directory.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/stores
  DESTINATION share/${PROJECT_NAME}/maps
)

# Shared runtime limits (single source of truth) live in the repository-level config/common.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
