cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14,
add_compile_options(-std=c++14)

project(noether_examples)

## Try to get VTK 7.1.  If that fails, try to get VTK 8.2
find_package(VTK 7.1 QUIET NO_MODULE)
if(${VTK_FOUND})
  include(${VTK_USE_FILE})
else()
  message("Could not find preferred VTK version (7.1).  Looking for other compatible version (8.2)")
  find_package(VTK 8.2 REQUIRED NO_MODULE)
  if(${VTK_FOUND})
    include(${VTK_USE_FILE})
  endif()
endif()

find_package(PCL 1.9 REQUIRED)

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
  ${PCL_INCLUDE_DIRS}
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

add_executable(mesh_filtering_client
  src/mesh_filtering_client.cpp
)
target_link_libraries(mesh_filtering_client
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(edge_generator_client
  src/edge_generator_client.cpp
)
target_link_libraries(edge_generator_client
  ${catkin_LIBRARIES}
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
