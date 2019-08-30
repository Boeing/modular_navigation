cmake_minimum_required(VERSION 2.8.3)
project(sim_band_planner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    gridmap
    nav_msgs
    navigation_interface
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include)

include_directories(SYSTEM
    ${Eigen3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    gridmap
    nav_msgs
    navigation_interface
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/band.h
    include/${PROJECT_NAME}/distance_field.h
    include/${PROJECT_NAME}/moving_window.h
    include/${PROJECT_NAME}/plugin.h
    include/${PROJECT_NAME}/simulate.h
    include/${PROJECT_NAME}/spline/Spline.h
    include/${PROJECT_NAME}/spline/SplineFitting.h
    include/${PROJECT_NAME}/spline/SplineFwd.h

    src/simulate.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(${PROJECT_NAME}_plugin
    src/plugin.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rosunit REQUIRED)
    catkin_add_gtest(unit_test test/unit_test.cpp)
    target_link_libraries(unit_test ${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
