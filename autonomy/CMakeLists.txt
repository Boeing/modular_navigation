cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(autonomy)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(gridmap REQUIRED)
find_package(map_manager REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(${PROJECT_NAME}_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono)

find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include ${YAML_CPP_INCLUDE_DIR})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# add_library(${PROJECT_NAME} SHARED src/autonomy.cpp )

add_executable(
    ${PROJECT_NAME} include/${PROJECT_NAME}/${PROJECT_NAME}.h include/${PROJECT_NAME}/math.h src/${PROJECT_NAME}.cpp
                    src/main.cpp
)
ament_target_dependencies(
    ${PROJECT_NAME}
    geometry_msgs
    gridmap
    map_manager
    map_msgs
    nav_msgs
    navigation_interface
    pluginlib
    tf2_ros
    urdf
    yaml-cpp
    tf2_geometry_msgs
    std_srvs
    ${PROJECT_NAME}_interface
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(PROGRAMS scripts/fake_localisation.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS launch/autonomy_launch.py DESTINATION share/${PROJECT_NAME})

# Install Cpp executables
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

#
# Add ros tests
# TODO, Conditional testing
if(BUILD_TESTING)
    enable_testing()
    find_file(NOSETESTS_EXECUTABLE nosetests)
    add_test(
        nosetests
        "${NOSETESTS_EXECUTABLE}"
        -v
        --with-xunit
    )

    install(DIRECTORY test/
            DESTINATION share/${PROJECT_NAME}/test)

    find_package(launch_testing_ament_cmake)
    add_launch_test(test/test_drive_to_waypoint.py)
endif()
ament_package()
