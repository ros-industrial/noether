cmake_minimum_required(VERSION 3.5.0)
project(noether_filtering VERSION 0.0.0 LANGUAGES CXX)

find_package(console_bridge REQUIRED)
find_package(PCL 1.9 REQUIRED COMPONENTS common filters io surface)
find_package(VTK 8.2 REQUIRED NO_MODULE)
find_package(class_loader REQUIRED)
find_package(xmlrpcpp REQUIRED)

###########
## Build ##
###########

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} SHARED
  src/filter_group.cpp
  src/filter_manager.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES}
  ${class_loader_LIBRARIES}
  ${PCL_LIBRARIES}
  ${xmlrpcpp_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${xmlrpcpp_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO)
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME} PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
endif()

# Mesh Filters library
add_library(${PROJECT_NAME}_mesh_filters SHARED
  src/mesh/bspline_reconstruction.cpp
)
target_link_libraries(${PROJECT_NAME}_mesh_filters PUBLIC ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_mesh_filters PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
set_target_properties(${PROJECT_NAME}_mesh_filters PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME}_mesh_filters PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME}_mesh_filters PRIVATE cxx_std_14)
endif()

# Cloud Filters library
add_library(${PROJECT_NAME}_cloud_filters SHARED src/cloud/filters.cpp)
target_link_libraries(${PROJECT_NAME}_cloud_filters PUBLIC ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_cloud_filters PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
set_target_properties(${PROJECT_NAME}_cloud_filters PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO)
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME}_cloud_filters PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME}_cloud_filters PRIVATE cxx_std_14)
endif()

# Filter Plugins Library
add_library(${PROJECT_NAME}_filter_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_filter_plugins PUBLIC
  ${PROJECT_NAME}_cloud_filters
  ${PROJECT_NAME}_mesh_filters
)
set_target_properties(${PROJECT_NAME}_filter_plugins PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME}_filter_plugins PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME}_filter_plugins PRIVATE cxx_std_14)
endif()

#############
## Install ##
#############
list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_mesh_filters ${PROJECT_NAME}_cloud_filters ${PROJECT_NAME}_filter_plugins)

# Install targets
install(TARGETS ${PACKAGE_LIBRARIES}
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION lib)

install(EXPORT ${PROJECT_NAME}-targets
  NAMESPACE noether_filtering:: DESTINATION lib/cmake/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install files
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

#############
## Testing ##
#############
if(${ENABLE_TESTS})
  find_package(GTest REQUIRED)
  add_executable(${PROJECT_NAME}_cloud_unit src/test/cloud_utest.cpp)
  set_target_properties(${PROJECT_NAME}_filter_plugins PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
  )
  target_link_libraries(${PROJECT_NAME}_cloud_unit PUBLIC
    ${PROJECT_NAME}_filter_plugins
    GTest::GTest
    GTest::Main
  )
  target_compile_options(${PROJECT_NAME}_cloud_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
  if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_cloud_unit PRIVATE -std=c++14)
  else()
    target_compile_features(${PROJECT_NAME}_cloud_unit PRIVATE cxx_std_14)
  endif()
  if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${PROJECT_NAME}_cloud_unit "" AUTO)
  else()
    gtest_discover_tests(${PROJECT_NAME}_cloud_unit)
  endif()
endif()
