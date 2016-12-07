cmake_minimum_required(VERSION 2.8.3)
project(noether)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cmake_modules
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer
    roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

FIND_PACKAGE(VTK 7.1 REQUIRED)

INCLUDE(${VTK_USE_FILE})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES noether

  CATKIN_DEPENDS
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer


  DEPENDS VTK
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
 add_library(noether
   src/noether.cpp
 )

 target_link_libraries(noether
   ${catkin_LIBRARIES}
   ${VTK_LIBRARIES}
 )
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(noether ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
 add_executable(noether_node src/noether.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(noether_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(noether_node
   noether
   ${catkin_LIBRARIES}
   ${VTK_LIBRARIES}
 )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_noether.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
