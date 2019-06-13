cmake_minimum_required(VERSION 2.8.3)
project(rrt_local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    geometry_msgs
    nav_msgs
    navigation_interface
    pluginlib
    roscpp
    rviz_visual_tools
    tf2_ros
)

find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

find_package(ompl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        costmap_2d
        geometry_msgs
        nav_msgs
        navigation_interface
        pluginlib
        roscpp
        rviz_visual_tools
        tf2_ros
    DEPENDS
        OMPL
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/pid.h
    include/${PROJECT_NAME}/${PROJECT_NAME}.h
    src/pid.cpp
    src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OMPL_LIBRARIES}
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
