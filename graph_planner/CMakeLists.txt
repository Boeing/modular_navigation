cmake_minimum_required(VERSION 3.0.2)
project(graph_planner)

find_package(catkin REQUIRED COMPONENTS geometry_msgs graph_map map_manager
                                        message_generation std_msgs)

catkin_python_setup()

# add_message_files(FILES )

add_service_files(FILES GraphPlan.srv LoadFloorplan.srv)

generate_messages(DEPENDENCIES geometry_msgs graph_map map_manager std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs graph_map map_manager
               message_runtime std_msgs)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
