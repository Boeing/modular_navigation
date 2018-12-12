cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    dynamic_reconfigure
    geometry_msgs
    laser_geometry
    map_msgs
    message_filters
    message_generation
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    tf2_sensor_msgs
    visualization_msgs
    voxel_grid
)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

include_directories(
    include
)

# messages
add_message_files(
    DIRECTORY msg
    FILES
    VoxelGrid.msg
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
        map_msgs
        std_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/Costmap2D.cfg
    cfg/GenericPlugin.cfg
    cfg/InflationPlugin.cfg
    cfg/ObstaclePlugin.cfg
    cfg/VoxelPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
        layers
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        laser_geometry
        map_msgs
        message_filters
        message_runtime
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros
        visualization_msgs
        voxel_grid
    DEPENDS
        Boost
        EIGEN3
)

add_library(${PROJECT_NAME}
    src/array_parser.cpp
    src/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}_publisher.cpp
    src/${PROJECT_NAME}_ros.cpp
    src/costmap_layer.cpp
    src/costmap_math.cpp
    src/footprint.cpp
    src/layer.cpp
    src/layered_costmap.cpp
    src/observation_buffer.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(layers
  src/plugins/inflation_layer.cpp
  src/plugins/obstacle_layer.cpp
  src/plugins/static_layer.cpp
  src/plugins/voxel_layer.cpp
)
add_dependencies(layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(layers
    ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_markers src/nodes/${PROJECT_NAME}_markers.cpp)
add_dependencies(${PROJECT_NAME}_markers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_markers
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_cloud src/nodes/${PROJECT_NAME}_cloud.cpp)
add_dependencies(${PROJECT_NAME}_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_cloud
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/nodes/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
  find_package(rostest REQUIRED)

  # Add the test folder to the include directories
  include_directories(test)

  # Create targets for test executables
  add_executable(costmap_tester EXCLUDE_FROM_ALL test/costmap_tester.cpp)
  add_dependencies(tests costmap_tester)
  target_link_libraries(costmap_tester ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(footprint_tests EXCLUDE_FROM_ALL test/footprint_tests.cpp)
  add_dependencies(tests footprint_tests)
  target_link_libraries(footprint_tests ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(obstacle_tests EXCLUDE_FROM_ALL test/obstacle_tests.cpp)
  add_dependencies(tests obstacle_tests)
  target_link_libraries(obstacle_tests ${PROJECT_NAME} layers ${GTEST_LIBRARIES})

  add_executable(static_tests EXCLUDE_FROM_ALL test/static_tests.cpp)
  add_dependencies(tests static_tests)
  target_link_libraries(static_tests ${PROJECT_NAME} layers ${GTEST_LIBRARIES})

  add_executable(inflation_tests EXCLUDE_FROM_ALL test/inflation_tests.cpp)
  add_dependencies(tests inflation_tests)
  target_link_libraries(inflation_tests ${PROJECT_NAME} layers ${GTEST_LIBRARIES})

  catkin_download_test_data(${PROJECT_NAME}_simple_driving_test_indexed.bag
    http://download.ros.org/data/${PROJECT_NAME}/simple_driving_test_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 61168cff9425b11e093ea3a627c81c8d)
  catkin_download_test_data(${PROJECT_NAME}_willow-full-0.025.pgm
    http://download.ros.org/data/${PROJECT_NAME}/willow-full-0.025.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 e66b17ee374f2d7657972efcb3e2e4f7)

  add_rostest(test/footprint_tests.launch)
  add_rostest(test/inflation_tests.launch)
  add_rostest(test/obstacle_tests.launch)
  add_rostest(test/simple_driving_test.xml)
  add_rostest(test/static_tests.launch)

  catkin_add_gtest(array_parser_test test/array_parser_test.cpp)
  target_link_libraries(array_parser_test ${PROJECT_NAME})
endif()

install( TARGETS
    ${PROJECT_NAME}_cloud
    ${PROJECT_NAME}_markers
    ${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    ${PROJECT_NAME}
    layers
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
