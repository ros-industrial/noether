find_gtest()

add_executable(${PROJECT_NAME}_tool_path_modifier_utest tool_path_modifier_utest.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_modifier_utest
  PRIVATE
    ${PROJECT_NAME}
    GTest::GTest
    GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_tool_path_modifier_utest)
add_dependencies(run_tests ${PROJECT_NAME}_tool_path_modifier_utest)

add_executable(${PROJECT_NAME}_tool_path_planner_utest tool_path_planner_utest.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_planner_utest
  PRIVATE
    ${PROJECT_NAME}
    GTest::GTest
    GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_tool_path_planner_utest)
add_dependencies(run_tests ${PROJECT_NAME}_tool_path_planner_utest)
target_compile_definitions(${PROJECT_NAME}_tool_path_planner_utest PUBLIC MESH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/meshes")

# Install the targets
install(
  TARGETS
    ${PROJECT_NAME}_tool_path_modifier_utest
    ${PROJECT_NAME}_tool_path_planner_utest
  RUNTIME DESTINATION bin/test
  LIBRARY DESTINATION lib/test
  ARCHIVE DESTINATION lib/test
)
