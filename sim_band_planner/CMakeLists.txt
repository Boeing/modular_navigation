cmake_minimum_required(VERSION 3.8)
project(sim_band_planner)

set(CMAKE_CXX_STANDARD 14)


set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#set(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_C_FLAGS}")

# add_definitions(-std=c++11)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(gridmap REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(map_manager REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread chrono)

# find_package( catkin REQUIRED COMPONENTS geometry_msgs gridmap nav_msgs navigation_interface pluginlib roscpp
# tf2_geometry_msgs tf2_ros visualization_msgs )

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include)

include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS}) # ${catkin_INCLUDE_DIRS}) ${Boost_INCLUDE_DIRS}

# catkin_package( CATKIN_DEPENDS geometry_msgs gridmap nav_msgs navigation_interface pluginlib roscpp tf2_geometry_msgs
# tf2_ros visualization_msgs INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

add_library(
    ${PROJECT_NAME}
    SHARED
    #include/${PROJECT_NAME}/band.h
    #include/${PROJECT_NAME}/distance_field.h
    #include/${PROJECT_NAME}/moving_window.h
    #include/${PROJECT_NAME}/plugin.h
    #include/${PROJECT_NAME}/simulate.h
    #include/${PROJECT_NAME}/spline/Spline.h
    #include/${PROJECT_NAME}/spline/SplineFitting.h
    #include/${PROJECT_NAME}/spline/SplineFwd.h
    src/simulate.cpp
    src/plugin.cpp
)
# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

# allow to find headers, see
# https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#adding-files-and-headers
target_include_directories(
    ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
    ${PROJECT_NAME}
    Eigen3
    Boost
    geometry_msgs
    nav_msgs
    visualization_msgs
    navigation_interface
    tf2_geometry_msgs
    tf2_ros
    gridmap
    pluginlib
)

#pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin.xml)
pluginlib_export_plugin_description_file(navigation_interface plugin.xml)

# target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# if(CATKIN_ENABLE_TESTING) find_package(rosunit REQUIRED) catkin_add_gtest(unit_test test/unit_test.cpp)
# target_link_libraries(unit_test ${PROJECT_NAME} ${OpenCV_LIBRARIES}) endif()

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME} # This one needed?
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
    PATTERN ".svn" EXCLUDE
)

# install( TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY
# DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()

# install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
