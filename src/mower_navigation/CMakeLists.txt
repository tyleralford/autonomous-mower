cmake_minimum_required(VERSION 3.8)
project(mower_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_bounds_guard.py
  scripts/cmd_vel_stamper.py
  DESTINATION lib/${PROJECT_NAME}
)

# Unit tests
if(BUILD_TESTING)
  ament_add_pytest_test(test_map_bounds_guard
    test/test_map_bounds_guard.py
  )
endif()

ament_package()
