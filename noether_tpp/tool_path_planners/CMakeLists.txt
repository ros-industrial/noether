# Add source files in this sub-directory
list(APPEND SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/raster_planner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/edge_planner.cpp)
set (SOURCES ${SOURCES} PARENT_SCOPE)

# Add headers in this sub-directory
list(APPEND HEADERS "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
set (HEADERS ${HEADERS} PARENT_SCOPE)

# Install headers
install(DIRECTORY include/ DESTINATION include/)
