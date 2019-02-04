cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14,
add_compile_options(-std=c++14)


project(noether_examples)


find_package(VTK 7.1 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED cmake_modules
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer
    noether_msgs
    roscpp
    pcl_conversions
    roslib
    actionlib
    noether_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES noether_examples
  CATKIN_DEPENDS
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer
    noether_msgs
    actionlib
    roslib
    roscpp
    pcl_conversions
  DEPENDS VTK
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})


add_executable(mesh_segmenter_client_node src/mesh_segmenter_client.cpp)

target_link_libraries(mesh_segmenter_client_node
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(mesh_segmenter_node src/mesh_segmenter_node.cpp)

target_link_libraries(mesh_segmenter_node
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)
