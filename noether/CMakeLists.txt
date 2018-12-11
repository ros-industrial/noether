cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14, 
add_compile_options(-std=c++14)

project(noether)

find_package(VTK 7.1 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED cmake_modules
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES noether
  CATKIN_DEPENDS
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer
  DEPENDS VTK
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(noether
    src/noether.cpp
)

target_link_libraries(noether
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(noether_node src/noether.cpp)

target_link_libraries(noether_node
    noether
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)
