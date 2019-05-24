cmake_minimum_required(VERSION 2.8.3)
project(map_manager)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
    hd_map
)

catkin_python_setup()

add_service_files(FILES
    AddMap.srv
    DeleteMap.srv
    GetMap.srv
    GetOccupancyGrid.srv
    ListMaps.srv
    SetActiveMap.srv
    UpdateMap.srv
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
        nav_msgs
        hd_map
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        nav_msgs
        hd_map
)

catkin_install_python(PROGRAMS
    scripts/${PROJECT_NAME}_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_manager.test)
endif()
