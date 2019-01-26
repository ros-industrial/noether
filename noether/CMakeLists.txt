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
    noether_msgs
    roscpp
    pcl_conversions
    actionlib
    noether_msgs
    noether_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES noether
  CATKIN_DEPENDS
    roscpp
    pcl_conversions
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer
    noether_msgs
    actionlib
  DEPENDS VTK
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(surface_raster_planner_server src/surface_raster_planner_server.cpp)
target_link_libraries(surface_raster_planner_server
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)

add_executable(surface_raster_planner_application src/surface_raster_planner_application.cpp)
target_link_libraries(surface_raster_planner_application
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)


add_executable(segmentation_server src/segmentation_server.cpp)
target_link_libraries(segmentation_server
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)
