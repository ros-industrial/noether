cmake_minimum_required(VERSION 2.8.3)
project(noether_conversions)

find_package(catkin REQUIRED
    cmake_modules
    geometry_msgs
    tool_path_planner
    tf
)

find_package(VTK 7.1 CONFIG)
find_package(VTK 7.1 REQUIRED)
include(${VTK_USE_FILE})

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    noether_conversions
  DEPENDS
    Eigen
    VTK
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

add_library(noether_conversions
    src/noether_conversions.cpp
)

target_link_libraries(noether_conversions
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)
