cmake_minimum_required(VERSION 2.8.3)
project(eband_local_planner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    geometry_msgs
    nav_msgs
    navigation_interface
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
)

find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include)

include_directories(SYSTEM
    ${Eigen3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
        costmap_2d
        geometry_msgs
        nav_msgs
        navigation_interface
        pluginlib
        roscpp
        tf2_geometry_msgs
        tf2_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/conversions_and_types.h
    include/${PROJECT_NAME}/${PROJECT_NAME}.h
    include/${PROJECT_NAME}/${PROJECT_NAME}_ros.h
    include/${PROJECT_NAME}/eband_trajectory_controller.h
    include/${PROJECT_NAME}/eband_visualization.h
    include/${PROJECT_NAME}/pid.h

    src/conversions_and_types.cpp
    src/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}_ros.cpp
    src/eband_trajectory_controller.cpp
    src/eband_visualization.cpp
    src/pid.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES elp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
