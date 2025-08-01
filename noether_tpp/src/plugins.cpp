#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>

// Mesh modifiers
#include <noether_tpp/mesh_modifiers/clean_data_modifier.h>
#include <noether_tpp/mesh_modifiers/compound_modifier.h>
#include <noether_tpp/mesh_modifiers/euclidean_clustering_modifier.h>
#include <noether_tpp/mesh_modifiers/fill_holes_modifier.h>
#include <noether_tpp/mesh_modifiers/normal_estimation_pcl.h>
#include <noether_tpp/mesh_modifiers/normals_from_mesh_faces_modifier.h>
#include <noether_tpp/mesh_modifiers/plane_projection_modifier.h>
#include <noether_tpp/mesh_modifiers/windowed_sinc_smoothing_modifier.h>

// Direction Generators
#include <noether_tpp/tool_path_planners/raster/direction_generators/fixed_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/pca_rotated_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>

// Origin Generators
#include <noether_tpp/tool_path_planners/raster/origin_generators/aabb_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/fixed_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/offset_origin_generator.h>

// Tool Path Planners
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/tool_path_planners/multi_tool_path_planner.h>

// Tool Path Modifiers
#include <noether_tpp/tool_path_modifiers/biased_tool_drag_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/circular_lead_in_modifier.h>
#include <noether_tpp/tool_path_modifiers/circular_lead_out_modifier.h>
#include <noether_tpp/tool_path_modifiers/concatenate_modifier.h>
#include <noether_tpp/tool_path_modifiers/direction_of_travel_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/fixed_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/linear_approach_modifier.h>
#include <noether_tpp/tool_path_modifiers/linear_departure_modifier.h>
#include <noether_tpp/tool_path_modifiers/moving_average_orientation_smoothing_modifier.h>
#include <noether_tpp/tool_path_modifiers/offset_modifier.h>
#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/snake_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/standard_edge_paths_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/tool_drag_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/uniform_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/uniform_spacing_linear_modifier.h>
#include <noether_tpp/tool_path_modifiers/uniform_spacing_spline_modifier.h>

namespace noether
{
// Mesh Modifiers
EXPORT_SIMPLE_MESH_MODIFIER_PLUGIN(CleanData, CleanData)
EXPORT_SIMPLE_MESH_MODIFIER_PLUGIN(EuclideanClusteringMeshModifier, EuclideanClustering)
EXPORT_SIMPLE_MESH_MODIFIER_PLUGIN(FillHoles, FillHoles)
EXPORT_SIMPLE_MESH_MODIFIER_PLUGIN(NormalEstimationPCLMeshModifier, NormalEstimationPCL)
EXPORT_SIMPLE_MESH_MODIFIER_PLUGIN(NormalsFromMeshFacesMeshModifier, NormalsFromMeshFaces)
EXPORT_SIMPLE_MESH_MODIFIER_PLUGIN(PlaneProjectionMeshModifier, PlaneProjection)
EXPORT_SIMPLE_MESH_MODIFIER_PLUGIN(WindowedSincSmoothing, WindowedSincSmoothing)

struct Plugin_CompoundMeshModifier : public Plugin<MeshModifier>
{
  std::unique_ptr<MeshModifier> create(const YAML::Node& config,
                                       std::shared_ptr<const Factory> factory) const override final
  {
    std::vector<MeshModifier::ConstPtr> modifiers;
    modifiers.reserve(config.size());

    auto modifiers_config = YAML::getMember<YAML::Node>(config, "modifiers");
    for (const YAML::Node& entry : modifiers_config)
    {
      auto name = YAML::getMember<std::string>(entry, "name");
      modifiers.push_back(factory->createMeshModifier(entry));
    }

    return std::make_unique<CompoundMeshModifier>(std::move(modifiers));
  }
};
EXPORT_MESH_MODIFIER_PLUGIN(Plugin_CompoundMeshModifier, CompoundMeshModifier)

// Direction Generators
EXPORT_SIMPLE_DIRECTION_GENERATOR_PLUGIN(FixedDirectionGenerator, FixedDirection)
EXPORT_SIMPLE_DIRECTION_GENERATOR_PLUGIN(PrincipalAxisDirectionGenerator, PrincipalAxis)

struct Plugin_PCARotatedDirectionGenerator : public Plugin<DirectionGenerator>
{
  std::unique_ptr<DirectionGenerator> create(const YAML::Node& config,
                                             std::shared_ptr<const Factory> factory) const override final
  {
    std::unique_ptr<DirectionGenerator> dir_gen;
    {
      auto dir_gen_config = YAML::getMember<YAML::Node>(config, "direction_generator");
      auto dir_gen_name = YAML::getMember<std::string>(dir_gen_config, "name");
      dir_gen = factory->createDirectionGenerator(dir_gen_config);
    }

    auto rotation_offset = YAML::getMember<double>(config, "rotation_offset");

    return std::make_unique<PCARotatedDirectionGenerator>(std::move(dir_gen), rotation_offset);
  }
};
EXPORT_DIRECTION_GENERATOR_PLUGIN(Plugin_PCARotatedDirectionGenerator, PCARotated)

// Origin Generators
EXPORT_SIMPLE_ORIGIN_GENERATOR_PLUGIN(AABBCenterOriginGenerator, AABBCenter)
EXPORT_SIMPLE_ORIGIN_GENERATOR_PLUGIN(CentroidOriginGenerator, Centroid)
EXPORT_SIMPLE_ORIGIN_GENERATOR_PLUGIN(FixedOriginGenerator, FixedOrigin)

struct Plugin_OffsetOriginGenerator : public Plugin<OriginGenerator>
{
  std::unique_ptr<OriginGenerator> create(const YAML::Node& config,
                                          std::shared_ptr<const Factory> factory) const override final
  {
    std::unique_ptr<OriginGenerator> origin_gen;
    {
      auto origin_gen_config = YAML::getMember<YAML::Node>(config, "origin_generator");
      auto origin_gen_name = YAML::getMember<std::string>(origin_gen_config, "name");
      origin_gen = factory->createOriginGenerator(origin_gen_config);
    }

    auto offset = YAML::getMember<Eigen::Vector3d>(config, "offset");

    return std::make_unique<OffsetOriginGenerator>(std::move(origin_gen), offset);
  }
};
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::Plugin_OffsetOriginGenerator, OffsetDirection)

// Tool Path Planners
struct Plugin_PlaneSlicerRasterPlanner : public Plugin<ToolPathPlanner>
{
  std::unique_ptr<ToolPathPlanner> create(const YAML::Node& config,
                                          std::shared_ptr<const Factory> factory) const override final
  {
    std::unique_ptr<DirectionGenerator> dir_gen;
    {
      auto dir_gen_config = YAML::getMember<YAML::Node>(config, "direction_generator");
      auto dir_gen_name = YAML::getMember<std::string>(dir_gen_config, "name");
      dir_gen = factory->createDirectionGenerator(dir_gen_config);
    }

    std::unique_ptr<OriginGenerator> origin_gen;
    {
      auto origin_gen_config = YAML::getMember<YAML::Node>(config, "origin_generator");
      auto origin_gen_name = YAML::getMember<std::string>(origin_gen_config, "name");
      origin_gen = factory->createOriginGenerator(origin_gen_config);
    }

    auto tpp = std::make_unique<PlaneSlicerRasterPlanner>(std::move(dir_gen), std::move(origin_gen));
    tpp->setLineSpacing(YAML::getMember<double>(config, "line_spacing"));
    tpp->setMinHoleSize(YAML::getMember<double>(config, "min_hole_size"));
    tpp->setPointSpacing(YAML::getMember<double>(config, "point_spacing"));
    tpp->setMinSegmentSize(YAML::getMember<double>(config, "min_segment_size"));
    tpp->setSearchRadius(YAML::getMember<double>(config, "search_radius"));
    tpp->generateRastersBidirectionally(YAML::getMember<bool>(config, "bidirectional"));

    return tpp;
  }
};
EXPORT_TOOL_PATH_PLANNER_PLUGIN(noether::Plugin_PlaneSlicerRasterPlanner, PlaneSlicer)

struct Plugin_MultiToolPathPlanner : public Plugin<ToolPathPlanner>
{
  std::unique_ptr<ToolPathPlanner> create(const YAML::Node& config,
                                          std::shared_ptr<const Factory> factory) const override final
  {
    std::vector<ToolPathPlanner::ConstPtr> tool_path_planners;
    tool_path_planners.reserve(config.size());

    auto planners_config = YAML::getMember<YAML::Node>(config, "planners");
    for (const YAML::Node& entry_config : planners_config)
    {
      auto entry_name = YAML::getMember<std::string>(entry_config, "name");
      tool_path_planners.push_back(factory->createToolPathPlanner(entry_config));
    }

    return std::make_unique<MultiToolPathPlanner>(std::move(tool_path_planners));
  }
};
EXPORT_TOOL_PATH_PLANNER_PLUGIN(noether::Plugin_MultiToolPathPlanner, Multi)

// Tool Path Modifiers
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(BiasedToolDragOrientationToolPathModifier, BiasedToolDragOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(CircularLeadInModifier, CircularLeadIn)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(CircularLeadOutModifier, CircularLeadOut)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(ConcatenateModifier, Concatenate)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(DirectionOfTravelOrientationModifier, DirectionOfTravelOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(FixedOrientationModifier, FixedOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(LinearApproachModifier, LinearApproach)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(LinearDepartureModifier, LinearDeparture)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(MovingAverageOrientationSmoothingModifier, MovingAverageOrientationSmoothing)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(OffsetModifier, Offset)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(RasterOrganizationModifier, RasterOrganization)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(SnakeOrganizationModifier, SnakeOrganization)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(StandardEdgePathsOrganizationModifier, StandardEdgePathsOrganization)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(ToolDragOrientationToolPathModifier, ToolDragOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(UniformOrientationModifier, UniformOrientation)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(UniformSpacingLinearModifier, UniformSpacingLinear)
EXPORT_SIMPLE_TOOL_PATH_MODIFIER_PLUGIN(UniformSpacingSplineModifier, UniformSpacingSpline)

}  // namespace noether

//! [Plugins Example Simple]
// Include the plugin interface headers
#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>

// Include the header for the custom tool path planning component
#include <noether_tpp/tool_path_planners/edge/boundary_edge_planner.h>

namespace noether
{
// For tool path planning components that can be fully configured by YAML serialization, invoke the
// `EXPORT_SIMPLE_<COMPONENT>_PLUGIN` macro. This macro instantiates the PluginImpl template for the input class name
// (first argument) and exports that plugin with the input arbitrary alias (second argument).
EXPORT_SIMPLE_TOOL_PATH_PLANNER_PLUGIN(BoundaryEdgePlanner, Boundary);

}  // namespace noether

//! [Plugins Example Simple]

//! [Plugins Example Complex]
// Include the plugin interface headers
#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>

// Include the header for the custom tool path planning component
#include <noether_tpp/tool_path_modifiers/compound_modifier.h>

namespace noether
{
// For a complex plugin that cannot be configured through YAML serialization alone, implement a custom plugin class.
struct Plugin_CompoundToolPathModifier : public Plugin<ToolPathModifier>
{
  std::unique_ptr<ToolPathModifier> create(const YAML::Node& config,
                                           std::shared_ptr<const Factory> factory) const override final
  {
    std::vector<ToolPathModifier::ConstPtr> modifiers;
    modifiers.reserve(config.size());

    auto modifiers_config = YAML::getMember<YAML::Node>(config, "modifiers");
    for (const YAML::Node& entry : modifiers_config)
    {
      auto entry_name = YAML::getMember<std::string>(entry, "name");
      modifiers.push_back(factory->createToolPathModifier(entry));
    }

    return std::make_unique<CompoundModifier>(std::move(modifiers));
  }
};

// Export the plugin class with an arbitrary alias using the `EXPORT_<COMPONENT>_PLUGIN` macro
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(Plugin_CompoundToolPathModifier, CompoundToolPathModifier);

}  // namespace noether

//! [Plugins Example Complex]
