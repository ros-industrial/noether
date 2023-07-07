cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(boost_plugin_loader REQUIRED)
find_package(noether_tpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS io surface)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(yaml-cpp REQUIRED)

option(NOETHER_ENABLE_CLANG_TIDY "Enables compilation with clang-tidy" OFF)
option(NOETHER_ENABLE_TESTING "Enables compilation of unit tests" OFF)

qt5_wrap_cpp(${PROJECT_NAME}_widget_mocs
  include/${PROJECT_NAME}/widgets/collapsible_area_widget.h
  include/${PROJECT_NAME}/widgets/tpp_pipeline_widget.h
  include/${PROJECT_NAME}/widgets/tpp_widget.h
  # Tool Path Planners
  #   Raster
  #   Direction Generator
  include/${PROJECT_NAME}/widgets/tool_path_planners/raster/direction_generators/fixed_direction_generator_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_planners/raster/direction_generators/principal_axis_direction_generator_widget.h
  #   Origin Generator
  include/${PROJECT_NAME}/widgets/tool_path_planners/raster/origin_generators/fixed_origin_generator_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_planners/raster/origin_generators/aabb_origin_generator_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_planners/raster/origin_generators/centroid_origin_generator_widget.h
  #   Plane Slicer Raster Planner
  include/${PROJECT_NAME}/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h
  # Tool Path Modifiers
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/circular_lead_in_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/circular_lead_out_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/direction_of_travel_orientation_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/fixed_orientation_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/moving_average_orientation_smoothing_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/raster_organization_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/snake_organization_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/standard_edge_paths_organization_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/tool_drag_orientation_modifier_widget.h
  include/${PROJECT_NAME}/widgets/tool_path_modifiers/uniform_orientation_modifier_widget.h)

qt5_wrap_ui(${PROJECT_NAME}_widget_ui_mocs
  ui/vector3d_editor_widget.ui
  ui/plugin_loader_widget.ui
  ui/raster_planner_widget.ui
  ui/tpp_pipeline_widget.ui
  ui/tpp_widget.ui)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

# Core Library
add_library(${PROJECT_NAME} SHARED
  src/plugin_interface.cpp
  # Widgets
  src/widgets/collapsible_area_widget.cpp
  src/widgets/plugin_loader_widget.cpp
  src/widgets/tpp_pipeline_widget.cpp
  src/widgets/tpp_widget.cpp
  # Tool Path Planners
  #   Raster Planner
  src/widgets/tool_path_planners/raster/raster_planner_widget.cpp
  #   Direction Generator
  src/widgets/tool_path_planners/raster/direction_generators/fixed_direction_generator_widget.cpp
  src/widgets/tool_path_planners/raster/direction_generators/principal_axis_direction_generator_widget.cpp
  #   Origin Generator
  src/widgets/tool_path_planners/raster/origin_generators/fixed_origin_generator_widget.cpp
  src/widgets/tool_path_planners/raster/origin_generators/aabb_origin_generator_widget.cpp
  src/widgets/tool_path_planners/raster/origin_generators/centroid_origin_generator_widget.cpp
  #   Plane Slicer Tool Path Planner
  src/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.cpp
  # Tool Path Modifiers
  src/widgets/tool_path_modifiers/circular_lead_in_modifier_widget.cpp
  src/widgets/tool_path_modifiers/circular_lead_out_modifier_widget.cpp
  src/widgets/tool_path_modifiers/direction_of_travel_orientation_modifier_widget.cpp
  src/widgets/tool_path_modifiers/fixed_orientation_modifier_widget.cpp
  src/widgets/tool_path_modifiers/moving_average_orientation_smoothing_modifier_widget.cpp
  src/widgets/tool_path_modifiers/raster_organization_modifier_widget.cpp
  src/widgets/tool_path_modifiers/snake_organization_modifier_widget.cpp
  src/widgets/tool_path_modifiers/standard_edge_paths_organization_modifier_widget.cpp
  src/widgets/tool_path_modifiers/tool_drag_orientation_modifier_widget.cpp
  src/widgets/tool_path_modifiers/uniform_orientation_modifier_widget.cpp
  ${${PROJECT_NAME}_widget_mocs}
  ${${PROJECT_NAME}_widget_ui_mocs}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} PUBLIC noether::noether_tpp Qt5::Widgets ${PCL_LIBRARIES} boost_plugin_loader::boost_plugin_loader Eigen3::Eigen yaml-cpp)
target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    NOETHER_GUI_PLUGINS="${PROJECT_NAME}_plugins"
    NOETHER_GUI_TPP_SECTION="tpp"
    NOETHER_GUI_DIRECTION_GENERATOR_SECTION="dg"
    NOETHER_GUI_ORIGIN_GENERATOR_SECTION="og"
    NOETHER_GUI_TOOL_PATH_MODIFIER_SECTION="mod"
    NOETHER_GUI_MESH_MODIFIER_SECTION="mesh"
    NOETHER_GUI_PLUGIN_LIBS_ENV="NOETHER_PLUGIN_LIBS"
    NOETHER_GUI_PLUGIN_PATHS_ENV="NOETHER_PLUGIN_PATHS"
)
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}
  ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${NOETHER_ENABLE_TESTING})

# Plugin Implementation Library
add_library(${PROJECT_NAME}_plugins SHARED
  src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME} Qt5::Widgets)
target_compile_definitions(${PROJECT_NAME}_plugins PRIVATE NOETHER_GUI_PLUGINS="${PROJECT_NAME}_plugins")
target_cxx_version(${PROJECT_NAME}_plugins PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME}_plugins
  ENABLE ${NOETHER_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${NOETHER_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME}_plugins INTERFACE ALL ENABLE ${NOETHER_ENABLE_TESTING})

# TPP Application
add_executable(${PROJECT_NAME}_app src/tpp_app.cpp)
target_link_libraries(${PROJECT_NAME}_app PRIVATE ${PROJECT_NAME} Boost::program_options)
target_compile_definitions(${PROJECT_NAME}_app PRIVATE NOETHER_GUI_PLUGINS="${PROJECT_NAME}_plugins")

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Package configuration
configure_package(NAMESPACE noether TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins ${PROJECT_NAME}_app)
