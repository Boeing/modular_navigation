cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    cv_bridge
    dynamic_reconfigure
    geometry_msgs
    laser_geometry
    map_msgs
    message_filters
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    tf2
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

generate_dynamic_reconfigure_options(
    cfg/Costmap2D.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
        layers
    CATKIN_DEPENDS
        cv_bridge
        dynamic_reconfigure
        geometry_msgs
        laser_geometry
        map_msgs
        message_filters
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros
        tf2_geometry_msgs
        visualization_msgs
    DEPENDS
        Boost
        EIGEN3
)

add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}_publisher.cpp
    src/${PROJECT_NAME}_ros.cpp
    src/costmap_layer.cpp
    src/costmap_math.cpp
    src/layer.cpp
    src/layered_costmap.cpp
    src/observation_buffer.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_library(layers
    src/plugins/inflation_layer.cpp
    src/plugins/obstacle_layer.cpp
    src/plugins/static_layer.cpp
)

add_dependencies(layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(layers
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
endif()

install(TARGETS
    ${PROJECT_NAME} layers
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)
