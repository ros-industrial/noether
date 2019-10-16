cmake_minimum_required(VERSION 3.5.0)
project(noether_filtering VERSION 0.1.0 LANGUAGES CXX)

find_package(console_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED NO_MODULE)
find_package(class_loader REQUIRED)
find_package(xmlrpcpp REQUIRED)

###########
## Build ##
###########

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} SHARED src/filter_manager.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES}
  ${class_loader_LIBRARIES}
  ${PCL_LIBRARIES}
  ${xmlrpcpp_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${xmlrpcpp_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO)
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME} PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
endif()

## Mesh Filters library
#add_library(${PROJECT_NAME}_mesh_filters SHARED src/filters/bspline_reconstruction.cpp)
#target_link_libraries(${PROJECT_NAME}_mesh_filters PUBLIC ${catkin_LIBRARIES})
#set_target_properties(${PROJECT_NAME}_mesh_filters PROPERTIES
#  CXX_STANDARD 14
#  CXX_STANDARD_REQUIRED YES
#  CXX_EXTENSIONS NO
#)
#target_include_directories(${PROJECT_NAME}_mesh_filters PUBLIC
#    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#    "$<INSTALL_INTERFACE:include>")
#target_include_directories(${PROJECT_NAME}_mesh_filters SYSTEM PUBLIC
#    ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(TARGETS ${PROJECT_NAME}_mesh_filters DESTINATION lib)

#############
## Testing ##
#############
