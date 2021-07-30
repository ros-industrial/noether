find_gtest()

add_executable(${PROJECT_NAME}_tool_path_modifier_utest tool_path_modifier_utest.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_modifier_utest
  PRIVATE
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_modifiers
    GTest::GTest
    GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_tool_path_modifier_utest)
add_dependencies(run_tests ${PROJECT_NAME}_tool_path_modifier_utest)

# Install the targets
install(
  TARGETS
    ${PROJECT_NAME}_tool_path_modifier_utest
  RUNTIME DESTINATION bin/test
  LIBRARY DESTINATION lib/test
  ARCHIVE DESTINATION lib/test
)
