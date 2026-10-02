cmake_minimum_required(VERSION 3.8)

project(serving_robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

ament_package()
