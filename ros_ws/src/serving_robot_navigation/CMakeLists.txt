cmake_minimum_required(VERSION 3.8)

project(serving_robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(serving_robot_navigation)

install(
  DIRECTORY
    config
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

install(DIRECTORY behavior_trees DESTINATION share/serving_robot_navigation)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/common DESTINATION share/serving_robot_navigation/config)
install(PROGRAMS scripts/navigation_executor.py scripts/dynamic_obstacles.py scripts/zone_masks.py DESTINATION lib/serving_robot_navigation)
ament_package()
