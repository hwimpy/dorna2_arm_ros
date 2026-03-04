cmake_minimum_required(VERSION 3.8)
project(dorna2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fixed_joint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
