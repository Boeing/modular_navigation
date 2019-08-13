cmake_minimum_required(VERSION 2.8.3)
project(omni_rrt_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    gridmap
    nav_msgs
    navigation_interface
    ompl
    pluginlib
    roscpp
    rviz_visual_tools
    sim_band_planner
)

find_package(OpenCV 3 REQUIRED)

find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omni_rrt_planner
  CATKIN_DEPENDS
    geometry_msgs
    gridmap
    nav_msgs
    navigation_interface
    pluginlib
    roscpp
    rviz_visual_tools
    sim_band_planner
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    include
)

add_library(omni_rrt_planner
    include/omni_rrt_planner/omni_rrt_planner.h
    include/omni_rrt_planner/rrt_star.h

    src/omni_rrt_planner.cpp
    src/rrt_star.cpp
)
target_link_libraries(omni_rrt_planner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS omni_rrt_planner
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
