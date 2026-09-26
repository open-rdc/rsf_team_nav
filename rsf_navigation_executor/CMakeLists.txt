cmake_minimum_required(VERSION 3.8)
project(rsf_navigation_executor)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps behavior_trees rviz waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_core.py
  scripts/waypoint_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/utils
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
