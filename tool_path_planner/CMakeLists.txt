cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14, 
add_compile_options(-std=c++14)

project(tool_path_planner)

find_package(VTK 7.1 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(PCL 1.8 REQUIRED)

find_package(catkin REQUIRED cmake_modules
    vtk_viewer
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES raster_tool_path_planner
    DEPENDS Eigen VTK vtk_viewer PCL
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(raster_tool_path_planner
    src/raster_tool_path_planner.cpp
    src/tool_path_planner.cpp
)

target_link_libraries(raster_tool_path_planner
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
    ${PCL_INCLUDE_DIRS}
)

catkin_add_gtest(raster_tool_path_planner-test test/utest.cpp)
target_link_libraries(raster_tool_path_planner-test
    raster_tool_path_planner
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)
