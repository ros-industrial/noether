cmake_minimum_required(VERSION 2.8.3)
project(path_sequence_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    vtk_viewer
    tool_path_planner
    #VTK
)

FIND_PACKAGE(VTK 7.1 REQUIRED PATHS /home/alex/path_planning_ws/src/VTK/ NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)

INCLUDE(${VTK_USE_FILE})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()





###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_sequence_planner
  CATKIN_DEPENDS
    tool_path_planner
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
 add_library(path_sequence_planner
   src/path_sequence_planner.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(path_planner_node src/path_planner_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(path_sequence_planner
   ${catkin_LIBRARIES}
   ${VTK_LIBRARIES}
 )



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
 catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
