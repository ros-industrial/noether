cmake_minimum_required(VERSION 2.8.3)
project(path_sequence_planner)

find_package(VTK 7.1 CONFIG)
find_package(VTK 7.1 REQUIRED)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED
    vtk_viewer
    tool_path_planner
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_path_sequence_planner
  CATKIN_DEPENDS
    tool_path_planner
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(simple_path_sequence_planner
    src/simple_path_sequence_planner.cpp
)

target_link_libraries(simple_path_sequence_planner
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
