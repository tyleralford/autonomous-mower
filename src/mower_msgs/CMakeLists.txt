cmake_minimum_required(VERSION 3.8)
project(mower_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavStatus.msg"
  "srv/ManageRecording.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
