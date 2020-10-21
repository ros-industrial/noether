cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14, 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tool_path_planner)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  eigen_stl_containers
  noether_conversions
  geometry_msgs
  noether_msgs
  rosconsole
  roslib
  shape_msgs
)

find_package(jsoncpp REQUIRED)
find_package(vtk_viewer REQUIRED)

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
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    eigen_conversions
    eigen_stl_containers
    geometry_msgs
    noether_msgs
    rosconsole
    shape_msgs
  DEPENDS
    EIGEN3
    PCL
    VTK
    vtk_viewer
    jsoncpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/surface_walk_raster_generator.cpp
  src/plane_slicer_raster_generator.cpp
  src/eigen_value_edge_generator.cpp
  src/halfedge_edge_generator.cpp
  src/utilities.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    noether::vtk_viewer
    jsoncpp_lib
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(raster_tool_path_planner-test
    test/utest.cpp
  )
  target_link_libraries(raster_tool_path_planner-test
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${VTK_LIBRARIES}
  )
  add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
  )
endif()
