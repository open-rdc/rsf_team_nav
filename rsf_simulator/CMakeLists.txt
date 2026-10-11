cmake_minimum_required(VERSION 3.8)
project(rsf_simulator)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds models config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/rsf_device_emulator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
