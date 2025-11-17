cmake_minimum_required(VERSION 3.8)
project(moveo_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
