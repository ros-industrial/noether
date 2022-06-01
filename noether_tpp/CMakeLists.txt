cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Create targets if necessary
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME} SHARED
  # Core
  src/core/tool_path_planner_pipeline.cpp
  # Tool Path Modifiers
  src/tool_path_modifiers/compound_modifier.cpp
  src/tool_path_modifiers/waypoint_orientation_modifiers.cpp
  src/tool_path_modifiers/organization_modifiers.cpp
  # Tool Path Planners
  src/tool_path_planners/edge_planner.cpp
  src/tool_path_planners/raster_planner.cpp
  src/tool_path_planners/direction_generators.cpp
  src/tool_path_planners/origin_generators.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}
  ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${NOETHER_ENABLE_TESTING})

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Unit tests
if(${NOETHER_ENABLE_TESTING})
  enable_testing()
  add_run_tests_target(ENABLE ${NOETHER_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Package configuration
configure_package(NAMESPACE noether TARGETS ${PROJECT_NAME})
