cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io surface segmentation)

find_package(VTK 7.1 REQUIRED NO_MODULE)

# Create targets if necessary
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME} SHARED
  # Core
  src/core/tool_path_planner_pipeline.cpp
  src/utils.cpp
  # Mesh Modifiers
  src/mesh_modifiers/subset_extraction/subset_extractor.cpp
  src/mesh_modifiers/subset_extraction/extruded_polygon_subset_extractor.cpp
  src/mesh_modifiers/compound_modifier.cpp
  src/mesh_modifiers/clean_data_modifier.cpp
  src/mesh_modifiers/euclidean_clustering_modifier.cpp
  src/mesh_modifiers/fill_holes_modifier.cpp
  src/mesh_modifiers/windowed_sinc_smoothing.cpp
  src/mesh_modifiers/plane_projection_modifier.cpp
  src/mesh_modifiers/normal_estimation_pcl.cpp
  src/mesh_modifiers/normals_from_mesh_faces_modifier.cpp
  # Tool Path Modifiers
  src/tool_path_modifiers/biased_tool_drag_orientation_modifier.cpp
  src/tool_path_modifiers/circular_lead_in_modifier.cpp
  src/tool_path_modifiers/circular_lead_out_modifier.cpp
  src/tool_path_modifiers/compound_modifier.cpp
  src/tool_path_modifiers/concatenate_modifier.cpp
  src/tool_path_modifiers/direction_of_travel_orientation_modifier.cpp
  src/tool_path_modifiers/fixed_orientation_modifier.cpp
  src/tool_path_modifiers/moving_average_orientation_smoothing_modifier.cpp
  src/tool_path_modifiers/raster_organization_modifier.cpp
  src/tool_path_modifiers/snake_organization_modifier.cpp
  src/tool_path_modifiers/standard_edge_paths_organization_modifier.cpp
  src/tool_path_modifiers/linear_approach_modifier.cpp
  src/tool_path_modifiers/linear_departure_modifier.cpp
  src/tool_path_modifiers/tool_drag_orientation_modifier.cpp
  src/tool_path_modifiers/uniform_orientation_modifier.cpp
  src/tool_path_modifiers/offset_modifier.cpp
  src/tool_path_modifiers/uniform_spacing_spline_modifier.cpp
  src/tool_path_modifiers/uniform_spacing_linear_modifier.cpp
  # Tool Path Planners
  src/tool_path_planners/multi_tool_path_planner.cpp
  src/tool_path_planners/edge/edge_planner.cpp
  src/tool_path_planners/edge/boundary_edge_planner.cpp
  src/tool_path_planners/raster/raster_planner.cpp
  src/tool_path_planners/raster/origin_generators/aabb_origin_generator.cpp
  src/tool_path_planners/raster/origin_generators/centroid_origin_generator.cpp
  src/tool_path_planners/raster/origin_generators/offset_origin_generator.cpp
  src/tool_path_planners/raster/origin_generators/fixed_origin_generator.cpp
  src/tool_path_planners/raster/direction_generators/fixed_direction_generator.cpp
  src/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.cpp
  src/tool_path_planners/raster/direction_generators/pca_rotated_direction_generator.cpp
  src/tool_path_planners/raster/plane_slicer_raster_planner.cpp
)

if(${CMAKE_VERSION} VERSION_GREATER 3.15)
    try_compile(
        NURBS_FOUND
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_modifiers/test/check_pcl_nurbs.cpp
        CMAKE_FLAGS ${CMAKE_FLAGS} -DINCLUDE_DIRECTORIES=${PCL_INCLUDE_DIRS}
        LINK_LIBRARIES pcl_surface
        )
    if (${NURBS_FOUND})
      target_sources(${PROJECT_NAME} PUBLIC src/mesh_modifiers/bspline_reconstruction_modifier.cpp)
    else()
      message("PCL NURBS not found; skipping compilation of BSpline reconstruction mesh modifier")
    endif()
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}
  ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${NOETHER_ENABLE_TESTING})

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Unit tests
if(${NOETHER_ENABLE_TESTING})
  enable_testing()
  add_run_tests_target(ENABLE ${NOETHER_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Package configuration
configure_package(NAMESPACE noether DEPENDENCIES Eigen3 "PCL REQUIRED COMPONENTS common io surface segmentation" VTK TARGETS ${PROJECT_NAME})
