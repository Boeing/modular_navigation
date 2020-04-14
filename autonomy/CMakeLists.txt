cmake_minimum_required(VERSION 2.8.3)
project(autonomy)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    cmake_modules
    geometry_msgs
    gridmap
    map_manager
    message_generation
    nav_msgs
    navigation_interface
    pluginlib
    roscpp
    std_srvs
    tf2_geometry_msgs
    tf2_ros
)
find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

find_package(yaml-cpp REQUIRED)

add_action_files(
    DIRECTORY
        action
    FILES
        Drive.action
)

generate_messages(DEPENDENCIES
    actionlib_msgs
    geometry_msgs
)

catkin_package(CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    map_manager
    message_runtime
    nav_msgs
    roscpp
    tf2_geometry_msgs
)

include_directories(
    include
    ${YAML_CPP_INCLUDE_DIR}
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    include/${PROJECT_NAME}/${PROJECT_NAME}.h
    include/${PROJECT_NAME}/math.h
    src/${PROJECT_NAME}.cpp
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

catkin_install_python(
    PROGRAMS
    scripts/fake_localisation.py
    DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
