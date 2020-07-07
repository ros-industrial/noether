cmake_minimum_required(VERSION 2.8.3)
project(mesh_segmenter)

## Compile as C++14,
add_compile_options(-std=c++14)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

find_package(catkin REQUIRED)
find_package(vtk_viewer REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    VTK
    vtk_viewer
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
    noether::vtk_viewer
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
  find_package(vtk_viewer REQUIRED)
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
