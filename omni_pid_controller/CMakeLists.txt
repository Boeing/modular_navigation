cmake_minimum_required(VERSION 2.8.3)
project(omni_pid_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    rviz_visual_tools
    tf2_ros
)

find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        costmap_2d
        geometry_msgs
        nav_core
        nav_msgs
        pluginlib
        roscpp
        rviz_visual_tools
        tf2_ros
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/${PROJECT_NAME}.h
    include/${PROJECT_NAME}/pid.h
    src/${PROJECT_NAME}.cpp
    src/pid.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
