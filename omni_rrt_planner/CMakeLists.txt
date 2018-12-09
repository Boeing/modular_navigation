cmake_minimum_required(VERSION 2.8.3)
project(omni_rrt_planner)

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
  LIBRARIES omni_rrt_planner
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

add_library(omni_rrt_planner
    include/omni_rrt_planner/omni_rrt_planner.h

    src/omni_rrt_planner.cpp
)
target_link_libraries(omni_rrt_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS omni_rrt_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(FILES omni_rrt_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
