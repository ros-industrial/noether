#include <boost_plugin_loader/plugin_loader.h>
#include <gtest/gtest.h>
#include <pcl/io/vtk_lib_io.h>

#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>
#include "utils.h"

/**
 * @brief YAML configuration string for the mesh modifiers
 */
std::string config_str = R"(
- name: CleanData
- &clustering
  name: EuclideanClustering
  tolerance: 1.0
  min_cluster_size: 1
  max_cluster_size: -1
- &fill_holes
  name: FillHoles
  max_hole_size: 5.0
- name: NormalEstimationPCL
  radius: 0.5
  vx: 0.0
  vy: 0.0
  vz: 10.0
- &normals_from_mesh_faces
  name: NormalsFromMeshFaces
- name: PlaneProjection
  distance_threshold: 2.5
  max_planes: 1
  min_vertices: 1
- name: WindowedSincSmoothing
  num_iter: 100
  enable_boundary_smoothing: true
  enable_feature_edge_smoothing: false
  enable_non_manifold_smoothing: true
  enable_normalize_coordinates: true
  feature_angle: 10.0
  edge_angle: 150.0
  pass_band: 0.01
- name: CompoundMeshModifier
  modifiers:
    - *fill_holes
    - *clustering
    - *normals_from_mesh_faces
)";

using namespace noether;

pcl::PolygonMesh loadWavyMeshWithHole()
{
  pcl::PolygonMesh mesh;
  const std::string mesh_file = MESH_DIR + std::string("/wavy_mesh_with_hole.ply");
  if (pcl::io::loadPolygonFile(mesh_file, mesh) > 0)
    return mesh;
  throw std::runtime_error("Failed to load test mesh from '" + mesh_file + "'");
}

/**
 * @brief Test fixture for all mesh modifiers
 */
class MeshModifierTestFixture : public testing::TestWithParam<std::shared_ptr<const MeshModifier>>
{
};

TEST_P(MeshModifierTestFixture, WavyMeshWithHole)
{
  auto modifier = GetParam();
  std::vector<pcl::PolygonMesh> meshes;
  ASSERT_NO_THROW(meshes = modifier->modify(loadWavyMeshWithHole()));
  ASSERT_FALSE(meshes.empty());
  for (const pcl::PolygonMesh& mesh : meshes)
  {
    EXPECT_FALSE(mesh.polygons.empty());
    EXPECT_GT(mesh.cloud.width * mesh.cloud.height, 0);
  }
}

// Create a vector of implementations for the modifiers
std::vector<std::shared_ptr<const MeshModifier>> createModifiers()
{
  // Create the plugin loader
  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(NOETHER_PLUGIN_LIB);
  loader.search_libraries_env = NOETHER_PLUGIN_LIBS_ENV;
  loader.search_paths_env = NOETHER_PLUGIN_PATHS_ENV;

  // Load the plugin YAML config
  YAML::Node config = YAML::Load(config_str);

  // Load the mesh modifiers
  std::vector<std::shared_ptr<const MeshModifier>> modifiers;
  modifiers.reserve(config.size());

  for (const YAML::Node& entry_config : config)
  {
    auto name = YAML::getMember<std::string>(entry_config, "name");
    auto plugin = loader.createInstance<MeshModifierPlugin>(name);
    modifiers.push_back(plugin->create(entry_config));
  }

  return modifiers;
}

INSTANTIATE_TEST_SUITE_P(MeshModifierTests,
                         MeshModifierTestFixture,
                         testing::ValuesIn(createModifiers()),
                         print<const MeshModifier>);

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
