cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 11)
project(navigation_interface)

#add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#find_package(catkin REQUIRED COMPONENTS geometry_msgs gridmap tf2_ros)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(gridmap REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

#catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS geometry_msgs gridmap tf2_ros)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

ament_package()