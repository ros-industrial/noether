cmake_minimum_required(VERSION 3.10.0)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

add_subdirectory(core)
add_subdirectory(tool_path_modifiers)
add_subdirectory(tool_path_planners)

# Unit tests
if(${NOETHER_ENABLE_TESTING})
  enable_testing()
  add_run_tests_target(ENABLE ${NOETHER_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Package configuration
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE "${PROJECT_NAME}::" DESTINATION lib/cmake/${PROJECT_NAME})
export(EXPORT ${PROJECT_NAME}-targets NAMESPACE "${PROJECT_NAME}::" FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
configure_package()
