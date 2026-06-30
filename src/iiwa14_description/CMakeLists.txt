cmake_minimum_required(VERSION 3.22)
project(iiwa14_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
