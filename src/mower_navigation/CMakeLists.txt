cmake_minimum_required(VERSION 3.8)
project(mower_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_bounds_guard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
