cmake_minimum_required(VERSION 3.0.2)
project(graph_map)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

catkin_python_setup()

add_message_files(FILES Region.msg Zone.msg)

# add_service_files(FILES )

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)

# catkin_install_python( PROGRAMS test/test_area.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

include_directories(${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_nosetests(src/${PROJECT_NAME}/test)
endif()
