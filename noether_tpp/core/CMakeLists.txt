# Add source files in this sub-directory
list(APPEND SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tool_path_planner_pipeline.cpp)
set (SOURCES ${SOURCES} PARENT_SCOPE)

# Add header files in this sub-directory
list(APPEND HEADERS "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
set (HEADERS ${HEADERS} PARENT_SCOPE)

# Install headers
install(DIRECTORY include/ DESTINATION include/)
