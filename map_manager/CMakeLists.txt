cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

find_package(catkin REQUIRED COMPONENTS geometry_msgs hd_map message_generation nav_msgs sensor_msgs)

catkin_python_setup()

add_service_files(
    FILES
    AddMap.srv
    DeleteMap.srv
    GetActiveMap.srv
    GetMap.srv
    GetOccupancyGrid.srv
    ListMaps.srv
    SetActiveMap.srv
    UpdateMap.srv
)

generate_messages(DEPENDENCIES geometry_msgs hd_map nav_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs hd_map message_runtime nav_msgs sensor_msgs)

catkin_install_python(
    PROGRAMS
    scripts/download_pbstream.py
    scripts/${PROJECT_NAME}_node.py
    scripts/save_cartographer_map.py
    scripts/save_gmapping_map.py
    scripts/template_world.py
    DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_manager.test)
endif()
