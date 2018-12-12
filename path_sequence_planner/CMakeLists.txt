cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14, 
add_compile_options(-std=c++14)

project(path_sequence_planner)

find_package(VTK 7.1 REQUIRED NO_MODULE)
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

catkin_add_gtest(simple_path_sequence_planner-test test/utest.cpp)
target_link_libraries(simple_path_sequence_planner-test
    simple_path_sequence_planner
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)

