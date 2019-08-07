cmake_minimum_required(VERSION 2.8.3)
project(noether_simulator)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(VTK 7.1 CONFIG)
find_package(VTK 7.1 REQUIRED)
include(${VTK_USE_FILE})

find_package(Eigen3 REQUIRED)

find_package(PCL 1.8 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED cmake_modules
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer
    roscpp
    dynamic_reconfigure
    actionlib
)

generate_dynamic_reconfigure_options(
  cfg/NoetherSimulator.cfg
)

catkin_package(
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    mesh_segmenter
    tool_path_planner
    path_sequence_planner
    vtk_viewer

    dynamic_reconfigure
  DEPENDS
    VTK
    PCL
)

include_directories(include
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/noether_simulator.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(coverage_sim src/coverage_simulator_node.cpp)
add_dependencies(coverage_sim ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${actionlib_tutorials_EXPORTED_TARGETS}
)
target_link_libraries(coverage_sim
    ${PROJECT_NAME}
)

#Test
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS vtk_viewer)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
  catkin_add_gtest(${PROJECT_NAME}-test
    test/utest.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} coverage_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
