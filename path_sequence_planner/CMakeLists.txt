cmake_minimum_required(VERSION 3.5.0)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH})

find_package(cmake_modules REQUIRED)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
endif()

find_package(vtk_viewer REQUIRED)

find_package(tool_path_planner REQUIRED)

add_library(simple_path_sequence_planner
  src/simple_path_sequence_planner.cpp
)
target_link_libraries(simple_path_sequence_planner
  ${VTK_LIBRARIES}
  noether::tool_path_planner
  noether::vtk_viewer
)
target_include_directories(simple_path_sequence_planner PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_cxx_version(simple_path_sequence_planner PUBLIC VERSION 14)
set_target_properties(simple_path_sequence_planner PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)

#############
## Install ##
#############
list(APPEND PACKAGE_LIBRARIES simple_path_sequence_planner)

configure_package(NAMESPACE noether TARGETS ${PACKAGE_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

#############
## Testing ##
#############
if(${ENABLE_TESTS})
  enable_testing()
  find_gtest()

  add_run_tests_target()

  add_executable(simple_path_sequence_planner-test test/utest.cpp)
  target_link_libraries(simple_path_sequence_planner-test PUBLIC
    ${PROJECT_NAME}
    ${VTK_LIBRARIES}
    GTest::GTest
    GTest::Main
  )
  target_cxx_version(simple_path_sequence_planner-tes PUBLIC VERSION 14)
  set_target_properties(simple_path_sequence_planner-tes PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)
  add_gtest_discover_tests(simple_path_sequence_planner-tes)

endif()
