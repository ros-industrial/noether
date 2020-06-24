cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14,
add_compile_options(-std=c++14)

project(noether_examples)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

find_package(PCL REQUIRED)
if(PCL_FOUND AND ("${PCL_VERSION}" VERSION_LESS 1.9))
  message(WARNING "The minimum required version of PCL is 1.9, but found ${PCL_VERSION} in path first. Checking for exactly 1.9")
  find_package(PCL REQUIRED 1.9)
else()
  add_definitions(${PCL_DEFINITIONS})
endif()

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
    PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(mesh_segmenter_client_node src/mesh_segmenter_client.cpp)
target_link_libraries(mesh_segmenter_client_node ${catkin_LIBRARIES} ${VTK_LIBRARIES})
list (APPEND PACKAGE_TARGETS mesh_segmenter_client_node)

add_executable(mesh_segmenter_node src/mesh_segmenter_node.cpp)
target_link_libraries(mesh_segmenter_node ${catkin_LIBRARIES} ${VTK_LIBRARIES})
list (APPEND PACKAGE_TARGETS mesh_segmenter_node)

add_executable(mesh_filtering_client src/mesh_filtering_client.cpp)
target_link_libraries(mesh_filtering_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})
list (APPEND PACKAGE_TARGETS mesh_filtering_client)

add_executable(edge_generator_client src/edge_generator_client.cpp)
target_link_libraries(edge_generator_client ${catkin_LIBRARIES})
list (APPEND PACKAGE_TARGETS edge_generator_client)

add_executable(halfedge_finder_node  src/halfedge_finder_node.cpp)
target_link_libraries(halfedge_finder_node ${catkin_LIBRARIES})
list (APPEND PACKAGE_TARGETS halfedge_finder_node)

#############
## Install ##
#############
install(TARGETS ${PACKAGE_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(
  DIRECTORY config launch data results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

