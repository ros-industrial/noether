# Add source files in this sub-directory
list(APPEND SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/compound_modifier.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/default_modifiers.cpp)
set (SOURCES ${SOURCES} PARENT_SCOPE)

# Add headers in this sub-directory
list(APPEND HEADERS "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
set (HEADERS ${HEADERS} PARENT_SCOPE)

# Install headers
install(DIRECTORY include/ DESTINATION include/)
