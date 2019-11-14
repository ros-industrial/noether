cmake_minimum_required(VERSION 2.8.3)
project(noether_conversions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  geometry_msgs
  noether_msgs
)

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

find_package(PCL 1.9 REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    noether_msgs
  DEPENDS
    EIGEN3
    VTK
    PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${PCL_LIBRARIES}
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
