cmake_minimum_required(VERSION 2.8.3)
project(map_io)

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    tf2
)

find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        nav_msgs
        roscpp
        tf2
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
    ${SDL_IMAGE_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib src/image_loader.cpp)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
)

add_executable(publish_map src/publish_map.cpp)
add_dependencies(publish_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_map
    ${PROJECT_NAME}_lib
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(save_map src/save_map.cpp)
add_dependencies(save_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_map
    ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_read_image
        test/unit_tests/test_read_image.cpp
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test/test_data
    )
    target_link_libraries(test_read_image
        ${PROJECT_NAME}_lib
        ${SDL_LIBRARY}
        ${SDL_IMAGE_LIBRARIES}
    )
endif()

install(TARGETS
    ${PROJECT_NAME}_lib
    publish_map
    save_map
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
