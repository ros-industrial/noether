find_package(PCL REQUIRED COMPONENTS common)

# Test Implementation Library
add_library(${PROJECT_NAME}_modifiers SHARED
  src/compound_modifier.cpp
  src/default_modifiers.cpp
)
target_include_directories(${PROJECT_NAME}_modifiers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_modifiers PUBLIC ${PROJECT_NAME}_core ${PCL_LIBRARIES})
target_cxx_version(${PROJECT_NAME}_modifiers PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}_modifiers
  ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME}_modifiers INTERFACE ALL ENABLE ${NOETHER_ENABLE_TESTING})

# Install the library
install(
  TARGETS ${PROJECT_NAME}_modifiers
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY include/ DESTINATION include/)
