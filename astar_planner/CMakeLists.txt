cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

add_definitions(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    geometry_msgs
    gridmap
    hd_map
    nav_msgs
    navigation_interface
    pluginlib
    roscpp
    tf2_ros
    visualization_msgs
)

find_package(OpenCV 3 REQUIRED)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS
      include
  LIBRARIES
      ${PROJECT_NAME}
  CATKIN_DEPENDS
      cv_bridge
      geometry_msgs
      gridmap
      hd_map
      nav_msgs
      navigation_interface
      pluginlib
      roscpp
      tf2_ros
      visualization_msgs
  DEPENDS
      EIGEN3
      OpenCV
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    src/astar.cpp
    src/costmap.cpp
    src/node.cpp
    src/plugin.cpp
    src/visualisation.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rosunit REQUIRED)

    catkin_add_gtest(unit_test test/unit_test.cpp)
    target_link_libraries(unit_test ${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
