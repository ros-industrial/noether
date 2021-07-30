cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.9 REQUIRED COMPONENTS common)

# Create targets if necessary
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

# Set up empty list variables for the source and header files from the sub-directories
set(SOURCES "")
set(HEADERS "")

add_subdirectory(core)
add_subdirectory(tool_path_modifiers)
add_subdirectory(tool_path_planners)

add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)
target_include_directories(${PROJECT_NAME} PUBLIC ${HEADERS} "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}
  ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${NOETHER_ENABLE_TESTING})

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Unit tests
if(${NOETHER_ENABLE_TESTING})
  enable_testing()
  add_run_tests_target(ENABLE ${NOETHER_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Package configuration
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE "noether::" DESTINATION lib/cmake/${PROJECT_NAME})
export(EXPORT ${PROJECT_NAME}-targets NAMESPACE "noether::" FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
configure_package()
