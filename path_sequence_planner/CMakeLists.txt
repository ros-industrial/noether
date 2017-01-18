cmake_minimum_required(VERSION 2.8.3)
project(path_sequence_planner)

find_package(VTK 7.1 REQUIRED)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED
    vtk_viewer
    tool_path_planner
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_sequence_planner
  CATKIN_DEPENDS
    tool_path_planner
)

add_library(path_sequence_planner
    src/path_sequence_planner.cpp
)

target_link_libraries(path_sequence_planner
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
