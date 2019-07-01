cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    gridmap
    navigation_interface
    pluginlib
    roscpp
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
        gridmap
        navigation_interface
        pluginlib
        roscpp
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/plugin.h
    src/plugin.cpp
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

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
