cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14, 
add_compile_options(-std=c++14)

project(tool_path_planner)
find_package(catkin REQUIRED cmake_modules vtk_viewer rosconsole noether_msgs shape_msgs geometry_msgs eigen_conversions)

find_package(VTK 7.1 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(PCL 1.8 REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS vtk_viewer rosconsole noether_msgs shape_msgs geometry_msgs eigen_conversions
    DEPENDS Eigen3 VTK PCL  
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/raster_tool_path_planner.cpp
    src/raster_path_generator.cpp
    src/utilities.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
)


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(raster_tool_path_planner-test test/utest.cpp)
  target_link_libraries(raster_tool_path_planner-test
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${VTK_LIBRARIES})
endif()


