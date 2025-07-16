#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <memory>

// Mesh modifiers
#include <noether_tpp/mesh_modifiers/clean_data_modifier.h>
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
using CleanDataMeshModifierPlugin = PluginImpl<CleanData, MeshModifier>;
using EuclideanClusteringMeshModifierPlugin = PluginImpl<EuclideanClusteringMeshModifier, MeshModifier>;
using FillHolesMeshModifierPlugin = PluginImpl<FillHoles, MeshModifier>;
using NormalEstimationPCLMeshModifierPlugin = PluginImpl<NormalEstimationPCLMeshModifier, MeshModifier>;
using NormalsFromMeshFacesMeshModifierPlugin = PluginImpl<NormalsFromMeshFacesMeshModifier, MeshModifier>;
using PlaneProjectionMeshModifierPlugin = PluginImpl<PlaneProjectionMeshModifier, MeshModifier>;
using WindowedSincSmoothingMeshModifierPlugin = PluginImpl<WindowedSincSmoothing, MeshModifier>;

// Direction Generators
using FixedDirectionGeneratorPlugin = PluginImpl<FixedDirectionGenerator, DirectionGenerator>;
using PrincipalAxisDirectionGeneratorPlugin = PluginImpl<PrincipalAxisDirectionGenerator, DirectionGenerator>;

struct PCARotatedDirectionGeneratorPlugin : public Plugin<DirectionGenerator>
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
using AABBCenterOriginGeneratorPlugin = PluginImpl<AABBCenterOriginGenerator, OriginGenerator>;
using CentroidOriginGeneratorPlugin = PluginImpl<CentroidOriginGenerator, OriginGenerator>;
using FixedOriginGeneratorPlugin = PluginImpl<FixedOriginGenerator, OriginGenerator>;

struct OffsetOriginGeneratorPlugin : public Plugin<OriginGenerator>
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
using BoundaryEdgePlannerPlugin = PluginImpl<BoundaryEdgePlanner, ToolPathPlanner>;

struct PlaneSlicerRasterPlannerPlugin : public Plugin<ToolPathPlanner>
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

// Tool Path Modifiers
using BiasedToolDragOrientationToolPathModifierPlugin =
    PluginImpl<BiasedToolDragOrientationToolPathModifier, ToolPathModifier>;
using CircularLeadInModifierPlugin = PluginImpl<CircularLeadInModifier, ToolPathModifier>;
using CircularLeadOutModifierPlugin = PluginImpl<CircularLeadOutModifier, ToolPathModifier>;
using ConcatenateModifierPlugin = PluginImpl<ConcatenateModifier, ToolPathModifier>;
using DirectionOfTravelOrientationModifierPlugin = PluginImpl<DirectionOfTravelOrientationModifier, ToolPathModifier>;
using FixedOrientationModifierPlugin = PluginImpl<FixedOrientationModifier, ToolPathModifier>;
using LinearApproachModifierPlugin = PluginImpl<LinearApproachModifier, ToolPathModifier>;
using LinearDepartureModifierPlugin = PluginImpl<LinearDepartureModifier, ToolPathModifier>;
using MovingAverageOrientationSmoothingModifierPlugin =
    PluginImpl<MovingAverageOrientationSmoothingModifier, ToolPathModifier>;
using OffsetModifierPlugin = PluginImpl<OffsetModifier, ToolPathModifier>;
using RasterOrganizationModifierPlugin = PluginImpl<RasterOrganizationModifier, ToolPathModifier>;
using SnakeOrganizationModifierPlugin = PluginImpl<SnakeOrganizationModifier, ToolPathModifier>;
using StandardEdgePathsOrganizationModifierPlugin = PluginImpl<StandardEdgePathsOrganizationModifier, ToolPathModifier>;
using ToolDragOrientationToolPathModifierPlugin = PluginImpl<ToolDragOrientationToolPathModifier, ToolPathModifier>;
using UniformOrientationModifierPlugin = PluginImpl<UniformOrientationModifier, ToolPathModifier>;
using UniformSpacingLinearModifierPlugin = PluginImpl<UniformSpacingLinearModifier, ToolPathModifier>;
using UniformSpacingSplineModifierPlugin = PluginImpl<UniformSpacingSplineModifier, ToolPathModifier>;

}  // namespace noether

// Mesh Modifiers
EXPORT_MESH_MODIFIER_PLUGIN(noether::CleanDataMeshModifierPlugin, CleanData)
EXPORT_MESH_MODIFIER_PLUGIN(noether::EuclideanClusteringMeshModifierPlugin, EuclideanClustering)
EXPORT_MESH_MODIFIER_PLUGIN(noether::FillHolesMeshModifierPlugin, FillHoles)
EXPORT_MESH_MODIFIER_PLUGIN(noether::NormalEstimationPCLMeshModifierPlugin, NormalEstimationPCL)
EXPORT_MESH_MODIFIER_PLUGIN(noether::NormalsFromMeshFacesMeshModifierPlugin, NormalsFromMeshFaces)
EXPORT_MESH_MODIFIER_PLUGIN(noether::PlaneProjectionMeshModifierPlugin, PlaneProjection)
EXPORT_MESH_MODIFIER_PLUGIN(noether::WindowedSincSmoothingMeshModifierPlugin, WindowedSincSmoothing)

// Direction Generators
EXPORT_DIRECTION_GENERATOR_PLUGIN(noether::FixedDirectionGeneratorPlugin, FixedDirection)
EXPORT_DIRECTION_GENERATOR_PLUGIN(noether::PrincipalAxisDirectionGeneratorPlugin, PrincipalAxis)
EXPORT_DIRECTION_GENERATOR_PLUGIN(noether::PCARotatedDirectionGeneratorPlugin, PCARotated)

// Origin Generators
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::AABBCenterOriginGeneratorPlugin, AABBCenter)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::CentroidOriginGeneratorPlugin, Centroid)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::FixedOriginGeneratorPlugin, FixedOrigin)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::OffsetOriginGeneratorPlugin, OffsetDirection)

// Tool Path Planners
EXPORT_TOOL_PATH_PLANNER_PLUGIN(noether::BoundaryEdgePlannerPlugin, Boundary)
EXPORT_TOOL_PATH_PLANNER_PLUGIN(noether::PlaneSlicerRasterPlannerPlugin, PlaneSlicer)

// Tool Path Modifiers
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::BiasedToolDragOrientationToolPathModifierPlugin, BiasedToolDragOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::CircularLeadInModifierPlugin, CircularLeadIn)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::CircularLeadOutModifierPlugin, CircularLeadOut)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::ConcatenateModifierPlugin, Concatenate)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::DirectionOfTravelOrientationModifierPlugin, DirectionOfTravelOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::FixedOrientationModifierPlugin, FixedOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::LinearApproachModifierPlugin, LinearApproach)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::LinearDepartureModifierPlugin, LinearDeparture)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::MovingAverageOrientationSmoothingModifierPlugin,
                                 MovingAverageOrientationSmoothing)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::OffsetModifierPlugin, Offset)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::RasterOrganizationModifierPlugin, RasterOrganization)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::SnakeOrganizationModifierPlugin, SnakeOrganization)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::StandardEdgePathsOrganizationModifierPlugin, StandardEdgePathsOrganization)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::ToolDragOrientationToolPathModifierPlugin, ToolDragOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::UniformOrientationModifierPlugin, UniformOrientation)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::UniformSpacingLinearModifierPlugin, UniformSpacingLinear)
EXPORT_TOOL_PATH_MODIFIER_PLUGIN(noether::UniformSpacingSplineModifierPlugin, UniformSpacingSpline)
