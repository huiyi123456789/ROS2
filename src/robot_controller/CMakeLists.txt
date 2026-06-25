cmake_minimum_required(VERSION 3.16)
project(robot_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Qt6 REQUIRED COMPONENTS Widgets Multimedia MultimediaWidgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(robot_controller_node
  src/main.cpp
  src/MainWindow.cpp
  src/VideoWidget.cpp
  src/RosCameraWidget.cpp
  src/MapWidget.cpp
  src/RobotController.cpp
  src/RobotControlWidget.cpp
  src/SpawnWidget.cpp
)

target_include_directories(robot_controller_node PRIVATE src)

target_link_libraries(robot_controller_node
  Qt6::Widgets
  Qt6::Multimedia
  Qt6::MultimediaWidgets
)

ament_target_dependencies(robot_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  gazebo_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS robot_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
