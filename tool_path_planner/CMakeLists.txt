cmake_minimum_required(VERSION 3.5.0)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(cmake_modules REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(vtk_viewer REQUIRED)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

find_package(PCL REQUIRED)
if(PCL_FOUND AND ("${PCL_VERSION}" VERSION_LESS 1.9))
  message(WARNING "The minimum required version of PCL is 1.9, but found ${PCL_VERSION} in path first. Checking for exactly 1.9")
  find_package(PCL REQUIRED 1.9)
else()
  add_definitions(${PCL_DEFINITIONS})
endif()
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/surface_walk_raster_generator.cpp
  src/plane_slicer_raster_generator.cpp
  src/eigen_value_edge_generator.cpp
  src/halfedge_edge_generator.cpp
  src/utilities.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    noether::vtk_viewer
    ${jsoncpp_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)
set_target_properties(${PROJECT_NAME} PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)

#############
## Install ##
#############
list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

configure_package(NAMESPACE noether TARGETS ${PACKAGE_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
#############
## Testing ##
#############
if(${ENABLE_TESTS})
  enable_testing()
  find_gtest()

  add_run_tests_target()

  add_executable(raster_tool_path_planner-test test/utest.cpp)
  target_link_libraries(raster_tool_path_planner-test PUBLIC
    ${PROJECT_NAME}
    ${VTK_LIBRARIES}
    GTest::GTest
    GTest::Main
  )
  target_cxx_version(raster_tool_path_planner-tes PUBLIC VERSION 14)
  set_target_properties(raster_tool_path_planner-tes PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)
  add_gtest_discover_tests(raster_tool_path_planner-tes)
  add_dependencies(run_tests ${PROJECT_NAME})
endif()
