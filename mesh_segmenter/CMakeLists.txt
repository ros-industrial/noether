cmake_minimum_required(VERSION 2.8.3)
project(mesh_segmenter)

## Compile as C++14,
add_compile_options(-std=c++14)

## Try to get VTK 7.1.  If that fails, try to get VTK 8.2
find_package(VTK 7.1 QUIET NO_MODULE)
if(${VTK_FOUND})
  include(${VTK_USE_FILE})
else()
  message("Could not find preferred VTK version (7.1).  Looking for other compatible version (8.2)")
  find_package(VTK 8.2 REQUIRED NO_MODULE)
  if(${VTK_FOUND})
    include(${VTK_USE_FILE})
  endif()
endif()

find_package(catkin REQUIRED)

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
)

add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

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
