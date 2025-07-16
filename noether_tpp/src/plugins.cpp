#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <memory>

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
#include <noether_tpp/tool_path_planners/edge/boundary_edge_planner.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/tool_path_planners/multi_tool_path_planner.h>

// Tool Path Modifiers
#include <noether_tpp/tool_path_modifiers/biased_tool_drag_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/circular_lead_in_modifier.h>
#include <noether_tpp/tool_path_modifiers/circular_lead_out_modifier.h>
#include <noether_tpp/tool_path_modifiers/compound_modifier.h>
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
static const boost_plugin_loader::PluginLoader& getPluginLoaderInstance()
{
  static boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert("noether_tpp_plugins");
  loader.search_libraries_env = NOETHER_PLUGIN_LIBS_ENV;
  loader.search_paths_env = NOETHER_PLUGIN_PATHS_ENV;
  return loader;
}

template <typename DerivedT, typename BaseT>
struct PluginImpl : public Plugin<BaseT>
{
  std::unique_ptr<BaseT> create(const YAML::Node& config = {}) const override final
  {
    return std::make_unique<DerivedT>(config.as<DerivedT>());
  }
};

// Mesh Modifiers
using Plugin_CleanDataMeshModifier = PluginImpl<CleanData, MeshModifier>;
using Plugin_EuclideanClusteringMeshModifier = PluginImpl<EuclideanClusteringMeshModifier, MeshModifier>;
using Plugin_FillHolesMeshModifier = PluginImpl<FillHoles, MeshModifier>;
using Plugin_NormalEstimationPCLMeshModifier = PluginImpl<NormalEstimationPCLMeshModifier, MeshModifier>;
using Plugin_NormalsFromMeshFacesMeshModifier = PluginImpl<NormalsFromMeshFacesMeshModifier, MeshModifier>;
using Plugin_PlaneProjectionMeshModifier = PluginImpl<PlaneProjectionMeshModifier, MeshModifier>;
using Plugin_WindowedSincSmoothingMeshModifier = PluginImpl<WindowedSincSmoothing, MeshModifier>;

struct Plugin_CompoundMeshModifier : public Plugin<MeshModifier>
{
  std::unique_ptr<MeshModifier> create(const YAML::Node& config = {}) const override final
  {
    const boost_plugin_loader::PluginLoader& loader = getPluginLoaderInstance();

    std::vector<MeshModifier::ConstPtr> modifiers;
    modifiers.reserve(config.size());

    auto modifiers_config = YAML::getMember<YAML::Node>(config, "modifiers");
    for (const YAML::Node& entry : modifiers_config)
    {
      auto entry_type = YAML::getMember<std::string>(entry, "type");
      auto entry_config = YAML::getMember<YAML::Node>(entry, "config");
      auto mesh_mod_plugin = loader.createInstance<MeshModifierPlugin>(entry_type);
      modifiers.push_back(mesh_mod_plugin->create(entry_config));
    }

    return std::make_unique<CompoundMeshModifier>(std::move(modifiers));
  }
};

// Direction Generators
using Plugin_FixedDirectionGenerator = PluginImpl<FixedDirectionGenerator, DirectionGenerator>;
using Plugin_PrincipalAxisDirectionGenerator = PluginImpl<PrincipalAxisDirectionGenerator, DirectionGenerator>;

struct Plugin_PCARotatedDirectionGenerator : public Plugin<DirectionGenerator>
{
  std::unique_ptr<DirectionGenerator> create(const YAML::Node& config = {}) const override final
  {
    std::unique_ptr<DirectionGenerator> dir_gen;
    {
      auto dir_gen_config = YAML::getMember<YAML::Node>(config, "direction_generator");
      auto dir_gen_type = YAML::getMember<std::string>(dir_gen_config, "type");
      auto dir_gen_params = YAML::getMember<YAML::Node>(dir_gen_config, "config");
      auto dir_gen_loader = getPluginLoaderInstance().createInstance<DirectionGeneratorPlugin>(dir_gen_type);
      dir_gen = dir_gen_loader->create(dir_gen_params);
    }

    auto rotation_offset = YAML::getMember<double>(config, "rotation_offset");

    return std::make_unique<PCARotatedDirectionGenerator>(std::move(dir_gen), rotation_offset);
  }
};

// Origin Generators
using Plugin_AABBCenterOriginGenerator = PluginImpl<AABBCenterOriginGenerator, OriginGenerator>;
using Plugin_CentroidOriginGenerator = PluginImpl<CentroidOriginGenerator, OriginGenerator>;
using Plugin_FixedOriginGenerator = PluginImpl<FixedOriginGenerator, OriginGenerator>;

struct Plugin_OffsetOriginGenerator : public Plugin<OriginGenerator>
{
  std::unique_ptr<OriginGenerator> create(const YAML::Node& config = {}) const override final
  {
    std::unique_ptr<OriginGenerator> origin_gen;
    {
      auto origin_gen_config = YAML::getMember<YAML::Node>(config, "origin_generator");
      auto origin_gen_type = YAML::getMember<std::string>(origin_gen_config, "type");
      auto origin_gen_params = YAML::getMember<YAML::Node>(origin_gen_config, "config");
      auto origin_gen_loader = getPluginLoaderInstance().createInstance<OriginGeneratorPlugin>(origin_gen_type);
      origin_gen = origin_gen_loader->create(origin_gen_params);
    }

    auto offset = YAML::getMember<Eigen::Vector3d>(config, "offset");

    return std::make_unique<OffsetOriginGenerator>(std::move(origin_gen), offset);
  }
};

// Tool Path Planners
using Plugin_BoundaryEdgePlanner = PluginImpl<BoundaryEdgePlanner, ToolPathPlanner>;

struct Plugin_PlaneSlicerRasterPlanner : public Plugin<ToolPathPlanner>
{
  std::unique_ptr<ToolPathPlanner> create(const YAML::Node& config = {}) const override final
  {
    std::unique_ptr<DirectionGenerator> dir_gen;
    {
      auto dir_gen_config = YAML::getMember<YAML::Node>(config, "direction_generator");
      auto dir_gen_type = YAML::getMember<std::string>(dir_gen_config, "type");
      auto dir_gen_params = YAML::getMember<YAML::Node>(dir_gen_config, "config");
      auto dir_gen_loader = getPluginLoaderInstance().createInstance<DirectionGeneratorPlugin>(dir_gen_type);
      dir_gen = dir_gen_loader->create(dir_gen_params);
    }

    std::unique_ptr<OriginGenerator> origin_gen;
    {
      auto origin_gen_config = YAML::getMember<YAML::Node>(config, "origin_generator");
      auto origin_gen_type = YAML::getMember<std::string>(origin_gen_config, "type");
      auto origin_gen_params = YAML::getMember<YAML::Node>(origin_gen_config, "config");
      auto origin_gen_loader = getPluginLoaderInstance().createInstance<OriginGeneratorPlugin>(origin_gen_type);
      origin_gen = origin_gen_loader->create(origin_gen_params);
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

struct Plugin_MultiToolPathPlanner : public Plugin<ToolPathPlanner>
{
  std::unique_ptr<ToolPathPlanner> create(const YAML::Node& config = {}) const override final
  {
    const boost_plugin_loader::PluginLoader& loader = getPluginLoaderInstance();

    std::vector<ToolPathPlanner::ConstPtr> tool_path_planners;
    tool_path_planners.reserve(config.size());

    auto planners_config = YAML::getMember<YAML::Node>(config, "planners");
    for (const YAML::Node& entry : planners_config)
    {
      auto entry_type = YAML::getMember<std::string>(entry, "type");
      auto entry_config = YAML::getMember<YAML::Node>(entry, "config");
      auto tpp_plugin = loader.createInstance<ToolPathPlannerPlugin>(entry_type);
      tool_path_planners.push_back(tpp_plugin->create(entry_config));
    }

    return std::make_unique<MultiToolPathPlanner>(std::move(tool_path_planners));
  }
};

// Tool Path Modifiers
using Plugin_BiasedToolDragOrientationToolPathModifier =
    PluginImpl<BiasedToolDragOrientationToolPathModifier, ToolPathModifier>;
using Plugin_CircularLeadInModifier = PluginImpl<CircularLeadInModifier, ToolPathModifier>;
using Plugin_CircularLeadOutModifier = PluginImpl<CircularLeadOutModifier, ToolPathModifier>;
using Plugin_ConcatenateModifier = PluginImpl<ConcatenateModifier, ToolPathModifier>;
using Plugin_DirectionOfTravelOrientationModifier = PluginImpl<DirectionOfTravelOrientationModifier, ToolPathModifier>;
using Plugin_FixedOrientationModifier = PluginImpl<FixedOrientationModifier, ToolPathModifier>;
using Plugin_LinearApproachModifier = PluginImpl<LinearApproachModifier, ToolPathModifier>;
using Plugin_LinearDepartureModifier = PluginImpl<LinearDepartureModifier, ToolPathModifier>;
using Plugin_MovingAverageOrientationSmoothingModifier =
    PluginImpl<MovingAverageOrientationSmoothingModifier, ToolPathModifier>;
using Plugin_OffsetModifier = PluginImpl<OffsetModifier, ToolPathModifier>;
using Plugin_RasterOrganizationModifier = PluginImpl<RasterOrganizationModifier, ToolPathModifier>;
using Plugin_SnakeOrganizationModifier = PluginImpl<SnakeOrganizationModifier, ToolPathModifier>;
using Plugin_StandardEdgePathsOrganizationModifier =
    PluginImpl<StandardEdgePathsOrganizationModifier, ToolPathModifier>;
using Plugin_ToolDragOrientationToolPathModifier = PluginImpl<ToolDragOrientationToolPathModifier, ToolPathModifier>;
using Plugin_UniformOrientationModifier = PluginImpl<UniformOrientationModifier, ToolPathModifier>;
using Plugin_UniformSpacingLinearModifier = PluginImpl<UniformSpacingLinearModifier, ToolPathModifier>;
using Plugin_UniformSpacingSplineModifier = PluginImpl<UniformSpacingSplineModifier, ToolPathModifier>;

struct Plugin_CompoundToolPathModifier : public Plugin<ToolPathModifier>
{
  std::unique_ptr<ToolPathModifier> create(const YAML::Node& config = {}) const override final
  {
    const boost_plugin_loader::PluginLoader& loader = getPluginLoaderInstance();

    std::vector<ToolPathModifier::ConstPtr> modifiers;
    modifiers.reserve(config.size());

    auto modifiers_config = YAML::getMember<YAML::Node>(config, "modifiers");
    for (const YAML::Node& entry : modifiers_config)
    {
      auto entry_name = YAML::getMember<std::string>(entry, "name");
      auto tool_path_mod_plugin = loader.createInstance<ToolPathModifierPlugin>(entry_name);
      modifiers.push_back(tool_path_mod_plugin->create(entry));
    }

    return std::make_unique<CompoundModifier>(std::move(modifiers));
  }
};

}  // namespace noether

// Mesh Modifiers
EXPORT_MESH_MODIFIER_PLUGIN(noether::Plugin_CleanDataMeshModifier, CleanData)
EXPORT_MESH_MODIFIER_PLUGIN(noether::Plugin_CompoundMeshModifier, CompoundMeshModifier)
EXPORT_MESH_MODIFIER_PLUGIN(noether::Plugin_EuclideanClusteringMeshModifier, EuclideanClustering)
EXPORT_MESH_MODIFIER_PLUGIN(noether::Plugin_FillHolesMeshModifier, FillHoles)
EXPORT_MESH_MODIFIER_PLUGIN(noether::Plugin_NormalEstimationPCLMeshModifier, NormalEstimationPCL)
EXPORT_MESH_MODIFIER_PLUGIN(noether::Plugin_NormalsFromMeshFacesMeshModifier, NormalsFromMeshFaces)
EXPORT_MESH_MODIFIER_PLUGIN(noether::Plugin_PlaneProjectionMeshModifier, PlaneProjection)
EXPORT_MESH_MODIFIER_PLUGIN(noether::Plugin_WindowedSincSmoothingMeshModifier, WindowedSincSmoothing)

// Direction Generators
EXPORT_DIRECTION_GENERATOR_PLUGIN(noether::Plugin_FixedDirectionGenerator, FixedDirection)
EXPORT_DIRECTION_GENERATOR_PLUGIN(noether::Plugin_PrincipalAxisDirectionGenerator, PrincipalAxis)
EXPORT_DIRECTION_GENERATOR_PLUGIN(noether::Plugin_PCARotatedDirectionGenerator, PCARotated)

// Origin Generators
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::Plugin_AABBCenterOriginGenerator, AABBCenter)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::Plugin_CentroidOriginGenerator, Centroid)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::Plugin_FixedOriginGenerator, FixedOrigin)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::Plugin_OffsetOriginGenerator, OffsetDirection)

// Tool Path Planners
EXPORT_TOOL_PATH_PLANNER_PLUGIN(noether::Plugin_BoundaryEdgePlanner, Boundary)
EXPORT_TOOL_PATH_PLANNER_PLUGIN(noether::Plugin_PlaneSlicerRasterPlanner, PlaneSlicer)
EXPORT_TOOL_PATH_PLANNER_PLUGIN(noether::Plugin_MultiToolPathPlanner, Multi)

// Tool Path Modifiers
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_BiasedToolDragOrientationToolPathModifier, BiasedToolDragOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_CircularLeadInModifier, CircularLeadIn)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_CircularLeadOutModifier, CircularLeadOut)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_CompoundToolPathModifier, CompoundToolPathModifier)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_ConcatenateModifier, Concatenate)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_DirectionOfTravelOrientationModifier, DirectionOfTravelOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_FixedOrientationModifier, FixedOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_LinearApproachModifier, LinearApproach)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_LinearDepartureModifier, LinearDeparture)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_MovingAverageOrientationSmoothingModifier,
                                 MovingAverageOrientationSmoothing)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_OffsetModifier, Offset)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_RasterOrganizationModifier, RasterOrganization)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_SnakeOrganizationModifier, SnakeOrganization)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_StandardEdgePathsOrganizationModifier, StandardEdgePathsOrganization)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_ToolDragOrientationToolPathModifier, ToolDragOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_UniformOrientationModifier, UniformOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_UniformSpacingLinearModifier, UniformSpacingLinear)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::Plugin_UniformSpacingSplineModifier, UniformSpacingSpline)
