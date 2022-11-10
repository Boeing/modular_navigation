cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(graph_map)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Package deps
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(message_runtime REQUIRED)

# Msg packages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Region.msg"
  "msg/Zone.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
