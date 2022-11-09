cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
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

#add_library(
#    ${PROJECT_NAME}
#    include/${PROJECT_NAME}/controller.h
#    include/${PROJECT_NAME}/params.h
#    include/${PROJECT_NAME}/path_planner.h
#    include/${PROJECT_NAME}/trajectory_planner.h
#    include/${PROJECT_NAME}/types/control.h
#    include/${PROJECT_NAME}/types/path.h
#    include/${PROJECT_NAME}/types/trajectory.h
#    src/file.cpp
#)
#catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS geometry_msgs gridmap tf2_ros)


# From:
# https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html#changing-the-cmake-code
include_directories(include)

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)

install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

ament_package()