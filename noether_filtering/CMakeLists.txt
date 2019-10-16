cmake_minimum_required(VERSION 2.8.3)
project(noether_filtering)

find_package(console_bridge REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(VTK 8.2 REQUIRED NO_MODULE)
find_package(catkin REQUIRED roscpp pluginlib)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_mesh_filters
  CATKIN_DEPENDS roscpp pluginlib
  DEPENDS console_bridge PCL VTK
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
# 	include
#  	${catkin_INCLUDE_DIRS}
# )

## Declare Main C++ library
add_library(${PROJECT_NAME} SHARED src/mesh_filter_manager.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES
	CXX_STANDARD 14
	CXX_STANDARD_REQUIRED YES
	CXX_EXTENSIONS NO
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


## Mesh Filters library
add_library(${PROJECT_NAME}_mesh_filters SHARED src/mesh_filters/bspline_reconstruction.cpp)
target_link_libraries(${PROJECT_NAME}_mesh_filters PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(${PROJECT_NAME}_mesh_filters PROPERTIES
	CXX_STANDARD 14
	CXX_STANDARD_REQUIRED YES
	CXX_EXTENSIONS NO
)
target_include_directories(${PROJECT_NAME}_mesh_filters PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_mesh_filters SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
   
   

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/noether_filtering_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME} DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_mesh_filters EXPORT ${PROJECT_NAME} DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_noether_filtering.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
