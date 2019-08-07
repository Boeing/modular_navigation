cmake_minimum_required(VERSION 2.8.3)
project(modular_move_base)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    cmake_modules
    geometry_msgs
    gridmap
    map_manager
    message_generation
    move_base_msgs
    nav_msgs
    navigation_interface
    pluginlib
    roscpp
    rviz_visual_tools
    std_srvs
    tf2_geometry_msgs
    tf2_ros
)
find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

add_service_files(FILES
    Plan.srv
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
        nav_msgs
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        map_manager
        message_runtime
        move_base_msgs
        nav_msgs
        roscpp
        tf2_geometry_msgs
)

include_directories(
    include
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    include/${PROJECT_NAME}/move_base.h
    src/main.cpp
    src/move_base.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
