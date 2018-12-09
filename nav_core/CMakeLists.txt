cmake_minimum_required(VERSION 2.8.3)
project(nav_core)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    geometry_msgs
    std_msgs
    tf2_ros
)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        costmap_2d
        geometry_msgs
        std_msgs
        tf2_ros
)


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
