cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
#cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

# Package deps
find_package(geometry_msgs REQUIRED)
find_package(graph_map REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#ament_python_install_package()

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_add_gtest(src/${PROJECT_NAME}/test/test_manager.test)
#endif()

ament_package()