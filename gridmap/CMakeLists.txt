cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(gridmap)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

find_package(cartographer_ros_msgs)
# find_package(cmake_modules REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_manager REQUIRED)
find_package(map_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(graph_map REQUIRED)
# find_package(roscpp REQUIRED) find_package(rospack REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rcutils REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

include_directories(include ${YAML_CPP_INCLUDE_DIR})

# catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS cartographer_ros_msgs cv_bridge
# geometry_msgs image_geometry laser_geometry map_manager map_msgs message_filters nav_msgs pluginlib roscpp sensor_msgs
# std_msgs tf2_geometry_msgs tf2_ros tf2_sensor_msgs urdf visualization_msgs DEPENDS Boost EIGEN3 )

add_library(
    ${PROJECT_NAME}
    SHARED
    src/grids/grid_2d.cpp
    src/grids/occupancy_grid.cpp
    src/grids/probability_grid.cpp
    src/layered_map.cpp
    src/layers/base_map_layer.cpp
    src/layers/obstacle_data/compressed_depth_data.cpp
    src/layers/obstacle_data/compressed_depth_image_transport/codec.cpp
    src/layers/obstacle_data/compressed_depth_image_transport/rvl_codec.cpp
    src/layers/obstacle_data/depth_data.cpp
    src/layers/obstacle_data/laser_data.cpp
    src/layers/obstacle_data/point_cloud_data.cpp
    src/layers/obstacle_data/range_data.cpp
    src/layers/obstacle_layer.cpp
    src/operations/clip_line.cpp
    src/operations/raytrace.cpp
    src/robot_tracker.cpp
)

# allow to find headers, see
# https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#adding-files-and-headers
target_include_directories(
    ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
    ${PROJECT_NAME}
    Eigen3
    cv_bridge
    Boost
    graph_map
    sensor_msgs
    rcutils
    cartographer_ros_msgs
    nav_msgs
    geometry_msgs
    map_manager
    image_geometry
)

# Link packages against new deps add_dependencies(${PROJECT_NAME}
# ${${PROJECT_NAME}_EXPORTED_TARGETS})#${catkin_EXPORTED_TARGETS}) target_link_libraries(${PROJECT_NAME}
# ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})

# if(CATKIN_ENABLE_TESTING) find_package(rosunit REQUIRED)
#
# catkin_add_gtest(unit_test test/unit_test.cpp) target_link_libraries(unit_test ${PROJECT_NAME} ${catkin_LIBRARIES}
# ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})
#
# catkin_add_gtest(test_raytrace test/test_raytrace.cpp) target_link_libraries(test_raytrace ${PROJECT_NAME}
# ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}) endif()

# Plugging Declaration
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME} # This one needed?
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    DIRECTORY include/
    DESTINATION include
    PATTERN ".svn" EXCLUDE
)

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
