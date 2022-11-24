cmake_minimum_required(VERSION 3.8)
project(astar_planner)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

#find_package(
#    catkin REQUIRED
#    COMPONENTS
#        cv_bridge
#        geometry_msgs
#        gridmap
#        nav_msgs
#        navigation_interface
#        pluginlib
#        roscpp
#        tf2_ros
#        visualization_msgs
#)

find_package(pluginlib REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gridmap REQUIRED)
find_package(navigation_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
# if not included, throws target not found when adding other libraries
find_package(graph_map REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(map_manager REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)



find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS} ${Boost_INCLUDE_DIRS})

#catkin_package(
#    INCLUDE_DIRS
#    include
#    LIBRARIES
#    ${PROJECT_NAME}
#    CATKIN_DEPENDS
#    cv_bridge
#    geometry_msgs
#    gridmap
#    nav_msgs
#    navigation_interface
#    pluginlib
#    roscpp
#    tf2_ros
#    visualization_msgs
#    DEPENDS
#    EIGEN3
#    OpenCV
#)

include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS})# ${catkin_INCLUDE_DIRS})

#include_directories(include)

add_library(
    ${PROJECT_NAME}
    src/astar.cpp
    src/costmap.cpp
    src/node.cpp src/plugin.cpp
    src/visualisation.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME} Eigen3 navigation_interface gridmap graph_map
    pluginlib cv_bridge geometry_msgs nav_msgs tf2_ros visualization_msgs OpenCV Boost)

#if(CATKIN_ENABLE_TESTING)
#    find_package(rosunit REQUIRED)
#
#    catkin_add_gtest(unit_test test/unit_test.cpp)
#    target_link_libraries(unit_test ${PROJECT_NAME} ${OpenCV_LIBRARIES})
#endif()

# Plugging Declaration
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin.xml)
    
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME} # This one needed?
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

#install(
#    DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    PATTERN ".svn" EXCLUDE
#)

ament_export_libraries(
    ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

#install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

ament_package()