find_package(PCL REQUIRED COMPONENTS common)

# Test Implementation Library
add_library(${PROJECT_NAME}_planners SHARED
  src/raster_planner.cpp
  src/edge_planner.cpp
)
target_include_directories(${PROJECT_NAME}_planners PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_planners PUBLIC ${PROJECT_NAME}_core ${PROJECT_NAME}_modifiers)
target_cxx_version(${PROJECT_NAME}_planners PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}_planners
  ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME}_planners INTERFACE ALL ENABLE ${NOETHER_ENABLE_TESTING})

# Install the library
install(
  TARGETS ${PROJECT_NAME}_planners
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY include/ DESTINATION include/)
