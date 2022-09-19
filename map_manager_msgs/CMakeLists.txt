cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(map_manager_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Package deps
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapInfo.msg"
  "srv/AddMap.srv"
  "srv/DeleteMap.srv"
  "srv/GetActiveMap.srv"
  "srv/GetAreaTree.srv"
  "srv/GetMapInfo.srv"
  "srv/GetNodeGraph.srv"
  "srv/GetOccupancyGrid.srv"
  "srv/GetZones.srv"
  "srv/ListMaps.srv"
  "srv/SetActiveMap.srv"
  DEPENDENCIES geometry_msgs nav_msgs sensor_msgs
)

ament_package()