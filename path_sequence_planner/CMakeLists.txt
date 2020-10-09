cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14, 
add_compile_options(-std=c++14)

project(path_sequence_planner)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

find_package(catkin REQUIRED COMPONENTS
    tool_path_planner
)

find_package(vtk_viewer REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    simple_path_sequence_planner
  CATKIN_DEPENDS
    tool_path_planner
  DEPENDS
    vtk_viewer
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_path_sequence_planner
  src/simple_path_sequence_planner.cpp
)
target_link_libraries(simple_path_sequence_planner
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  noether::vtk_viewer
)

#############
## Install ##
#############
install(TARGETS simple_path_sequence_planner
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
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(simple_path_sequence_planner-test
    test/utest.cpp
  )
  target_link_libraries(simple_path_sequence_planner-test
    simple_path_sequence_planner
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
  )
endif()
