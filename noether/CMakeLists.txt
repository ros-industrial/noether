cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14, 
add_compile_options(-std=c++14)

project(noether)

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
)

include_directories(include ${catkin_INCLUDE_DIRS} ${noether_filtering_INCLUDE_DIRS})

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
  ${noether_filtering_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS segmentation_server surface_raster_planner_application surface_raster_planner_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES config/tool.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
