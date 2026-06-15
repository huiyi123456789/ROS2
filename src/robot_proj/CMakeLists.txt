cmake_minimum_required(VERSION 3.8)
project(robot_proj)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  models
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
