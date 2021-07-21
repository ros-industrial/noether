# Find dependency packages
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

# Create targets if necessary
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

# Interface Library
add_library(${PROJECT_NAME}_core INTERFACE)
target_include_directories(${PROJECT_NAME}_core INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_core INTERFACE Eigen3::Eigen ${PCL_LIBRARIES})
target_cxx_version(${PROJECT_NAME}_core INTERFACE VERSION 14)
target_clang_tidy(${PROJECT_NAME}_core
  ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})

# Tool Path Planner Pipeline Library
add_library(${PROJECT_NAME}_planner_pipeline SHARED
  src/tool_path_planner_pipeline.cpp)
target_include_directories(${PROJECT_NAME}_planner_pipeline PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_planner_pipeline PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_cxx_version(${PROJECT_NAME}_planner_pipeline PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}_planner_pipeline   ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME}_planner_pipeline INTERFACE ALL ENABLE ${NOETHER_ENABLE_TESTING})

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_planner_pipeline
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/)
