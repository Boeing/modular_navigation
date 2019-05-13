cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    cv_bridge
    geometry_msgs
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
        cv_bridge
        geometry_msgs
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
        visualization_msgs
    DEPENDS
        Boost
        EIGEN3
)

add_library(${PROJECT_NAME}
    src/map_data.cpp
    src/map_publisher.cpp
    src/plugins/base_map_data.cpp
    src/plugins/laser_data.cpp
    src/plugins/point_cloud_data.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(costmap
    src/main.cpp
)

target_link_libraries(costmap
    ${PROJECT_NAME}
)

add_dependencies(costmap
    ${PROJECT_NAME}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
endif()

install(TARGETS
    ${PROJECT_NAME} costmap
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
