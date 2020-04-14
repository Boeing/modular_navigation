cmake_minimum_required(VERSION 3.1)
project(gridmap)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    cartographer_ros_msgs
    cmake_modules
    compressed_depth_image_transport
    cv_bridge
    geometry_msgs
    hd_map
    image_geometry
    laser_geometry
    map_msgs
    message_filters
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2_sensor_msgs
    urdf
    visualization_msgs
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

include_directories(
    include
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        cartographer_ros_msgs
        compressed_depth_image_transport
        cv_bridge
        geometry_msgs
        hd_map
        image_geometry
        laser_geometry
        map_msgs
        message_filters
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf2_geometry_msgs
        tf2_ros
        tf2_sensor_msgs
        urdf
        visualization_msgs
    DEPENDS
        Boost
        EIGEN3
)

add_library(${PROJECT_NAME}
    src/grids/grid_2d.cpp
    src/grids/occupancy_grid.cpp
    src/grids/probability_grid.cpp
    src/layered_map.cpp
    src/layers/base_map_layer.cpp
    src/layers/obstacle_data/compressed_depth_data.cpp
    src/layers/obstacle_data/depth_data.cpp
    src/layers/obstacle_data/laser_data.cpp
    src/layers/obstacle_data/point_cloud_data.cpp
    src/layers/obstacle_data/range_data.cpp
    src/layers/obstacle_layer.cpp
    src/operations/clip_line.cpp
    src/operations/raytrace.cpp
    src/robot_tracker.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rosunit REQUIRED)

    catkin_add_gtest(unit_test test/unit_test.cpp)
    target_link_libraries(unit_test ${PROJECT_NAME} ${OpenCV_LIBRARIES})

    catkin_add_gtest(test_raytrace test/test_raytrace.cpp)
    target_link_libraries(test_raytrace ${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)
