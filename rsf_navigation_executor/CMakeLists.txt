cmake_minimum_required(VERSION 3.8)
project(rsf_navigation_executor)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch config behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
