cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14,
add_compile_options(-std=c++14)

project(noether)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

find_package(PCL 1.9 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  mesh_segmenter
  noether_conversions
  noether_msgs
  path_sequence_planner
  pcl_conversions
  roscpp
  tool_path_planner
  vtk_viewer
)

find_package(noether_filtering REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    actionlib
    mesh_segmenter
    noether_conversions
    noether_msgs
    path_sequence_planner
    pcl_conversions
    roscpp
    tool_path_planner
    vtk_viewer
  DEPENDS
    VTK
    PCL
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(surface_raster_planner_server
  src/surface_raster_planner_server.cpp
)
target_link_libraries(surface_raster_planner_server
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(surface_raster_planner_application
  src/surface_raster_planner_application.cpp
)
target_link_libraries(surface_raster_planner_application
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(segmentation_server
  src/segmentation_server.cpp
)
target_link_libraries(segmentation_server
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(mesh_filter_server
  src/mesh_filter_server.cpp
)
target_link_libraries(mesh_filter_server PUBLIC
  ${catkin_LIBRARIES}
  noether_filtering::noether_filtering
)

#############
## Install ##
#############
install(TARGETS segmentation_server surface_raster_planner_application surface_raster_planner_server mesh_filter_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
