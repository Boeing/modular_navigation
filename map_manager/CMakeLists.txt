cmake_minimum_required(VERSION 2.8.3)
project(map_manager)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
)

catkin_python_setup()

add_message_files(FILES
    Map.msg
    Marker.msg
    Markers.msg
    Zone.msg
    Zones.msg
)

add_service_files(FILES
    AddMap.srv
    AddMarker.srv
    AddZone.srv
    DeleteMap.srv
    DeleteMarker.srv
    DeleteZone.srv
    GetMap.srv
    GetMarker.srv
    GetOccupancyGrid.srv
    GetZone.srv
    ListMaps.srv
    ListMarkers.srv
    ListZones.srv
    SetActiveMap.srv
    UpdateMap.srv
    UpdateMarker.srv
    UpdateZone.srv
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
        nav_msgs
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        nav_msgs
)

catkin_install_python(PROGRAMS
    scripts/${PROJECT_NAME}_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_manager.test)
endif()
