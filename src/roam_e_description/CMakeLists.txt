cmake_minimum_required(VERSION 3.8)
project(roam_e_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/preview_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME preview_joint_state_publisher
)

ament_package()
