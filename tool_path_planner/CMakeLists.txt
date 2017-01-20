cmake_minimum_required(VERSION 2.8.3)
project(tool_path_planner)

find_package(VTK 7.1 CONFIG)
find_package(VTK 7.1 REQUIRED)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED cmake_modules
    vtk_viewer
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES raster_tool_path_planner
    DEPENDS Eigen VTK
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(raster_tool_path_planner
    src/raster_tool_path_planner.cpp
    src/tool_path_planner.cpp
)

target_link_libraries(raster_tool_path_planner
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
