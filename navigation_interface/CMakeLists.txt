cmake_minimum_required(VERSION 2.8.3)
project(navigation_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    geometry_msgs
    std_msgs
    tf2_ros
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        costmap_2d
        geometry_msgs
        std_msgs
        tf2_ros
    DEPENDS
        Eigen
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
