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
  tool_path_planner
)

find_package(vtk_viewer REQUIRED)

find_package(noether_filtering)
if(NOT noether_filtering_FOUND)
  message(WARNING "noether_filtering not found. mesh_filter_server will not be compiled")
endif()

catkin_package(
  LIBRARIES
    mesh_segmenter
  CATKIN_DEPENDS
    actionlib
    mesh_segmenter
    noether_conversions
    noether_msgs
    path_sequence_planner
    pcl_conversions
    roscpp
    tool_path_planner
  DEPENDS
    VTK
    PCL
    vtk_viewer
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tool_path_planner_server
  src/tool_path_planner_server.cpp
)

target_link_libraries(tool_path_planner_server
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  noether::vtk_viewer
)

add_executable(segmentation_server
  src/segmentation_server.cpp
)
target_link_libraries(segmentation_server
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  noether::vtk_viewer
)
#There needs to be a better pathing to the library
add_executable(convex_hull_node
              src/convex_hull_node.cpp
              ../mesh_segmenter/src/convex_hull.cpp
)

target_link_libraries(convex_hull_node  
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  noether::vtk_viewer
)

if(noether_filtering_FOUND)
  add_executable(mesh_filter_server
    src/mesh_filter_server.cpp
  )
  target_link_libraries(mesh_filter_server PUBLIC
    ${catkin_LIBRARIES}
    noether_filtering::noether_filtering
    noether::vtk_viewer
  )
endif()

#############
## Install ##
#############
install(TARGETS segmentation_server tool_path_planner_server convex_hull_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(noether_filtering_FOUND)
  install(TARGETS mesh_filter_server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
