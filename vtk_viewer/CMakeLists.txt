cmake_minimum_required(VERSION 2.8.3)

## Compile as C++14, 
add_compile_options(-std=c++14)

project(vtk_viewer)

find_package(catkin REQUIRED cmake_modules rosconsole)

find_package(VTK 7.1 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(PCL 1.8 REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vtk_viewer
  DEPENDS
    Eigen
    PCL
    VTK
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(vtk_viewer
    src/vtk_viewer.cpp
    src/vtk_utils.cpp
    src/mouse_interactor.cpp
)

target_link_libraries(vtk_viewer
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
