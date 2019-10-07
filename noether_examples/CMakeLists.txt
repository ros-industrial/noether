cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14,
add_compile_options(-std=c++14)

project(noether_examples)

find_package(VTK 8.2 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  mesh_segmenter
  noether_conversions
  noether_msgs
  path_sequence_planner
  pcl_conversions
  roscpp
  roslib
  tool_path_planner
  vtk_viewer
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    mesh_segmenter
    noether_msgs
    path_sequence_planner
    pcl_conversions
    roscpp
    roslib
    tool_path_planner
    vtk_viewer
  DEPENDS
    VTK
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mesh_segmenter_client_node
  src/mesh_segmenter_client.cpp
)
target_link_libraries(mesh_segmenter_client_node
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(mesh_segmenter_node
  src/mesh_segmenter_node.cpp
)
target_link_libraries(mesh_segmenter_node
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS mesh_segmenter_client_node mesh_segmenter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
