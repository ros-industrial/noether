cmake_minimum_required(VERSION 2.8.3)
project(noether_examples)


#find_package(PCL 1.8 REQUIRED COMPONENTS common io)
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


find_package(VTK 7.1 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED cmake_modules
    mesh_segmenter
#    tool_path_planner
#    path_sequence_planner
    vtk_viewer
    noether
    noether_msgs
    roscpp
    roslib
    actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES noether
  CATKIN_DEPENDS
    mesh_segmenter
#    tool_path_planner
#    path_sequence_planner
    vtk_viewer
    noether
    noether_msgs
    roslib
    actionlib
  DEPENDS VTK
)

include_directories(
    #include
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
