cmake_minimum_required(VERSION 2.8.3)
project(rrt_local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_ros
    rviz_visual_tools
)

find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

find_package(OMPL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt_local_planner
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_ros
    rviz_visual_tools
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories(
    include
)

add_library(rrt_local_planner
    include/rrt_local_planner/pid.h
    include/rrt_local_planner/rrt_local_planner.h

    src/rrt_local_planner.cpp
    src/pid.cpp
)
target_link_libraries(rrt_local_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS rrt_local_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(FILES rrt_local_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
