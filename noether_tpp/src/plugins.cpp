#include <noether_tpp/plugin_interface.h>
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
#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>

// Origin Generators
#include <noether_tpp/tool_path_planners/raster/origin_generators/aabb_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/fixed_origin_generator.h>

namespace noether
{
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

// Origin Generators
using AABBCenterOriginGeneratorPlugin = PluginImpl<AABBCenterOriginGenerator, OriginGenerator>;
using CentroidOriginGeneratorPlugin = PluginImpl<CentroidOriginGenerator, OriginGenerator>;
using FixedOriginGeneratorPlugin = PluginImpl<FixedOriginGenerator, OriginGenerator>;

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

// Origin Generators
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::AABBCenterOriginGeneratorPlugin, AABBCenter)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::CentroidOriginGeneratorPlugin, Centroid)
EXPORT_ORIGIN_GENERATOR_PLUGIN(noether::FixedOriginGeneratorPlugin, FixedOrigin)
