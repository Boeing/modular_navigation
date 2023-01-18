cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
project(pure_pursuit_controller)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# cmake_policy(SET CMP0057 NEW)

find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(gridmap REQUIRED)
find_package(navigation_interface REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
# if not included, throws target not found when adding other libraries
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(graph_map REQUIRED)
find_package(map_manager REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)

add_definitions(${EIGEN3_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system thread chrono)
# include_directories(include)
include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS}) # ${catkin_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# catkin_package( CATKIN_DEPENDS +

# navigation_interface pluginlib roscpp INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

# Followed https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html#cmake-plugin-declaration
# add_library( ${PROJECT_NAME} src/plugin.cpp )

add_library(${PROJECT_NAME}
    SHARED
    #include/${PROJECT_NAME}/plugin.h
    src/plugin.cpp
)

target_include_directories(
    ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} Eigen3 navigation_interface gridmap pluginlib)

# allow to find headers, see
# https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#adding-files-and-headers

# Plugging Declaration
pluginlib_export_plugin_description_file(navigation_interface plugin.xml)

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
# install( DIRECTORY include/ DESTINATION include PATTERN ".svn" EXCLUDE )

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_export_include_directories(include)
# install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

ament_package()
