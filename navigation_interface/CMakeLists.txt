cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(navigation_interface)

# add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find_package(catkin REQUIRED COMPONENTS geometry_msgs gridmap tf2_ros)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(gridmap REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(xmlrpcpp REQUIRED)

# adding an exec for testing purposes DEBUG add_executable(file_test src/file_test.cpp)
# target_include_directories(file_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
#
# ament_target_dependencies( file_test "pluginlib" )
#
# END DEBUG

# From: https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html#changing-the-cmake-code
# include_directories(include)

install(DIRECTORY include/ DESTINATION include)

# install(TARGETS file_test DESTINATION lib/${PROJECT_NAME}) # DEBUG

ament_export_include_directories(include)

# install( DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE )

ament_package()
