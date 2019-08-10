cmake_minimum_required(VERSION 2.8.3)
project(mesh_segmenter)

## Compile as C++14,
add_compile_options(-std=c++14)

find_package(
  VTK 7.1 REQUIRED NO_MODULE
)
include(${VTK_USE_FILE})

find_package(catkin REQUIRED COMPONENTS
    roscpp)

find_package(PCL 1.8 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    VTK
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    src/convex_hull.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}

)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

###############
## Exectable ##
###############
add_executable(convex_hull_node
          src/convex_hull.cpp)

target_link_libraries(convex_hull_node ${PROJECT_NAME})
#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(
  PROGRAMS 
    convex_hull_node
  DESTINATION    
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)



#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
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
