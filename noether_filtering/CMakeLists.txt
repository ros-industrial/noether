cmake_minimum_required(VERSION 3.5.0)
project(noether_filtering VERSION 0.1.0)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
else()
  include(${VTK_USE_FILE})
  set(BUILD_MESH_PLUGINS TRUE)
endif()

find_package(PCL REQUIRED COMPONENTS common filters surface segmentation)
if(PCL_FOUND AND ("${PCL_VERSION}" VERSION_LESS 1.9))
  message(WARNING "The minimum required version of PCL is 1.9, but found ${PCL_VERSION} in path first. Checking for exactly 1.9")
  find_package(PCL REQUIRED 1.9 COMPONENTS common filters surface segmentation)
else()
  add_definitions(${PCL_DEFINITIONS})
endif()
find_package(console_bridge REQUIRED)
find_package(pluginlib REQUIRED)
find_package(xmlrpcpp REQUIRED)

# Check if PCL was compiled with OpenNURBS
if(NOT DEFINED NURBS_FOUND)
    try_compile(
        NURBS_FOUND
        ${CMAKE_CURRENT_BINARY_DIR}/pcl_nurbs_try_compile
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_scripts/check_pcl_nurbs.cpp
        CMAKE_FLAGS ${CMAKE_FLAGS} -DINCLUDE_DIRECTORIES=${PCL_INCLUDE_DIRS}
        LINK_LIBRARIES ${PCL_LIBRARIES}
        )
    if (NOT ${NURBS_FOUND})
        message("PCL NURBS NOT FOUND")
    else()
        message("PCL NURBS FOUND")
    endif()
endif()

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
  ${pluginlib_LIBRARIES}
  ${PCL_LIBRARIES}
  ${xmlrpcpp_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${xmlrpcpp_INCLUDE_DIRS}
)
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME} PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)

if(${BUILD_MESH_PLUGINS})
  # Mesh Filters library
  message("Building mesh plugins")
  set(MESH_FILTERS_SRC
      src/mesh/euclidean_clustering.cpp
      src/mesh/clean_data.cpp
      src/mesh/fill_holes.cpp
      src/mesh/windowed_sinc_smoothing.cpp
      )
  set(MESH_FILTERS_COMPILE_DEFINITIONS "")

if (${NURBS_FOUND})
    list(APPEND MESH_FILTERS_SRC src/mesh/bspline_reconstruction.cpp)
    list(APPEND MESH_FILTERS_COMPILE_DEFINITIONS ENABLE_NURBS)
endif()

  add_library(${PROJECT_NAME}_mesh_filters SHARED ${MESH_FILTERS_SRC})
  target_compile_definitions(${PROJECT_NAME}_mesh_filters PUBLIC ${MESH_FILTERS_COMPILE_DEFINITIONS})


  target_link_libraries(${PROJECT_NAME}_mesh_filters PUBLIC ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_mesh_filters PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_mesh_filters PRIVATE -std=c++14)
  else()
    target_compile_features(${PROJECT_NAME}_mesh_filters PRIVATE cxx_std_14)
  endif()
  set_target_properties(${PROJECT_NAME}_mesh_filters PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)

  # Mesh Filter Plugins Library
  add_library(${PROJECT_NAME}_mesh_filter_plugins SHARED src/mesh/plugins.cpp)
  target_link_libraries(${PROJECT_NAME}_mesh_filter_plugins PUBLIC
    ${PROJECT_NAME}_mesh_filters
  )
  if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_mesh_filter_plugins PRIVATE -std=c++14)
  else()
    target_compile_features(${PROJECT_NAME}_mesh_filter_plugins PRIVATE cxx_std_14)
  endif()
  set_target_properties(${PROJECT_NAME}_mesh_filter_plugins PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)
endif()

# Cloud Filters library
add_library(${PROJECT_NAME}_cloud_filters SHARED src/cloud/filters.cpp)
target_link_libraries(${PROJECT_NAME}_cloud_filters PUBLIC ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_cloud_filters PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME}_cloud_filters PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME}_cloud_filters PRIVATE cxx_std_14)
endif()
set_target_properties(${PROJECT_NAME}_cloud_filters PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)

# Cloud Filter Plugins Library
add_library(${PROJECT_NAME}_cloud_filter_plugins SHARED src/cloud/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_cloud_filter_plugins PUBLIC
  ${PROJECT_NAME}_cloud_filters
)
if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME}_cloud_filter_plugins PRIVATE -std=c++14)
else()
  target_compile_features(${PROJECT_NAME}_cloud_filter_plugins PRIVATE cxx_std_14)
endif()
set_target_properties(${PROJECT_NAME}_cloud_filter_plugins PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)

#############
## Install ##
#############
if(${BUILD_MESH_PLUGINS})
  list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_mesh_filters ${PROJECT_NAME}_mesh_filter_plugins ${PROJECT_NAME}_cloud_filters ${PROJECT_NAME}_cloud_filter_plugins)
else()
  list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_cloud_filters ${PROJECT_NAME}_cloud_filter_plugins)
endif()

# Install targets
install(TARGETS ${PACKAGE_LIBRARIES}
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION lib)

install(EXPORT ${PROJECT_NAME}-targets
  NAMESPACE noether_filtering:: DESTINATION lib/cmake/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install files
install(FILES package.xml cloud_filter_plugins.xml mesh_filter_plugins.xml
  DESTINATION share/${PROJECT_NAME})

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

# Allows Colcon to find non-Ament packages when using workspace underlays
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv "prepend-non-duplicate;ROS_PACKAGE_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv DESTINATION share/${PROJECT_NAME}/hook)

#############
## Testing ##
#############
if(${ENABLE_TESTS})

  # Download and unpack googletest at configure time
  configure_file(cmake/CMakeLists.txt.in googletest-download/CMakeLists.txt)
  execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
      WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download"
  )
  execute_process(COMMAND "${CMAKE_COMMAND}" --build .
      WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download"
  )

  # Prevent GoogleTest from overriding our compiler/linker options
  # when building with Visual Studio
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  # Add googletest directly to our build. This adds the following targets:
  # gtest, gtest_main, gmock and gmock_main
  add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
                   "${CMAKE_BINARY_DIR}/googletest-build"
  )

  # The gtest/gmock targets carry header search path dependencies
  # automatically when using CMake 2.8.11 or later. Otherwise we
  # have to add them here ourselves.
  if(CMAKE_VERSION VERSION_LESS 2.8.11)
      include_directories("${gtest_SOURCE_DIR}/include"
                          "${gmock_SOURCE_DIR}/include"
      )
  endif()

  add_executable(${PROJECT_NAME}_cloud_unit src/test/cloud_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_cloud_unit PUBLIC
    ${PROJECT_NAME}_cloud_filter_plugins
    gtest_main
  )
  target_compile_options(${PROJECT_NAME}_cloud_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
  if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_cloud_unit PRIVATE -std=c++14)
  else()
    target_compile_features(${PROJECT_NAME}_cloud_unit PRIVATE cxx_std_14)
  endif()
set_target_properties(${PROJECT_NAME}_cloud_unit PROPERTIES NO_SYSTEM_FROM_IMPORTED 1)
endif()
