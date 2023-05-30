cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
# cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Package deps
find_package(geometry_msgs REQUIRED)
find_package(graph_map REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(map_manager_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

rosidl_generate_interfaces(
    ${PROJECT_NAME}
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
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    sensor_msgs
    graph_map
)

install(PROGRAMS ${PROJECT_NAME}/map_manager_node.py DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS launch/test_manager.py DESTINATION share/${PROJECT_NAME})

# if (BUILD_TESTING) find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(src/${PROJECT_NAME}/test/test_manager.test) endif()

ament_package()
