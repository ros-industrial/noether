cmake_minimum_required(VERSION 2.8.3)
project(mesh_segmenter)

find_package(VTK 7.1 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mesh_segmenter
  DEPENDS VTK
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mesh_segmenter
    src/mesh_segmenter.cpp
)

target_link_libraries(mesh_segmenter
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
