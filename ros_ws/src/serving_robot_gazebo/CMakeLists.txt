cmake_minimum_required(VERSION 3.8)

project(serving_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    worlds
    models
    launch
    config
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

ament_package()
