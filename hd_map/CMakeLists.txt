cmake_minimum_required(VERSION 2.8.3)
project(hd_map)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
)

add_message_files(FILES
    Map.msg
    MapInfo.msg
    Marker.msg
    Node.msg
    Path.msg
    Zone.msg
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
