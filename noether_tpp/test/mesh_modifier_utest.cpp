#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>
#include "utils.h"

#include <filesystem>
#include <gtest/gtest.h>
#include <pcl/io/vtk_lib_io.h>

/**
 * @brief YAML configuration string for mesh modifiers that retain the full set of data fields of the input mesh vertex
 * cloud
 */
static const std::string mesh_modifiers_data_retaining = R"(
- name: CleanData
- &clustering
  name: EuclideanClustering
  tolerance: 1.0
  min_cluster_size: 1
  max_cluster_size: -1
- name: FaceMidpointSubdivision
  n_iterations: 2
- name: FaceSubdivisionByArea
  max_area: 0.01
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
- name: RansacCylinderProjection
  distance_threshold: 1.0
  max_primitives: 1
  min_vertices: 1
  max_iterations: 100
  min_radius: 0.1
  max_radius: 10.0
  axis:
    x: 0.0
    y: 0.0
    z: 1.0
  axis_threshold: 3.14159
  normal_distance_weight: 0.015
- name: RansacPlaneProjection
  distance_threshold: 2.5
  max_primitives: 1
  min_vertices: 1
  max_iterations: 100
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

/**
 * @brief YAML configuration string for mesh modifiers that do not retain the full set of data fields of the input mesh
 * vertex cloud
 */
static const std::string mesh_modifiers_data_mutating = R"(
- name: RansacCylinderFit
  distance_threshold: 1.0
  max_primitives: 1
  min_vertices: 1
  max_iterations: 100
  min_radius: 0.1
  max_radius: 10.0
  axis:
    x: 0.0
    y: 0.0
    z: 1.0
  axis_threshold: 3.14159
  normal_distance_weight: 0.015
  resolution: 50
  include_caps: false
  uniform_triangles: true
)";

/** @brief A combination of all mesh modifier configurations to test */
static const std::string all_mesh_modifiers = mesh_modifiers_data_mutating + mesh_modifiers_data_retaining;

using namespace noether;

pcl::PolygonMesh loadMesh(const std::string& mesh_file)
{
  pcl::PolygonMesh mesh;
  if (pcl::io::loadPolygonFile(mesh_file, mesh) > 0)
    return mesh;
  throw std::runtime_error("Failed to load test mesh from '" + mesh_file + "'");
}

bool operator==(const pcl::PCLPointField& lhs, const pcl::PCLPointField& rhs)
{
  bool equivalent = true;
  equivalent &= lhs.count == rhs.count;
  equivalent &= lhs.datatype == rhs.datatype;
  equivalent &= lhs.name == rhs.name;
  equivalent &= lhs.offset == rhs.offset;
  return equivalent;
}

using MeshModifierTestParams = std::tuple<std::string, std::shared_ptr<const MeshModifier>>;

/** @brief Prints name of class for unit test output */
std::string printMeshModifierTestParams(const testing::TestParamInfo<MeshModifierTestParams> info)
{
  std::string mesh_file;
  std::shared_ptr<const MeshModifier> modifier;
  std::tie(mesh_file, modifier) = info.param;

  return std::to_string(info.index) + "_" + mesh_file.substr(0, mesh_file.find(".")) + "_" + getClassName(*modifier);
}

using MeshModifierTestFixture = testing::TestWithParam<MeshModifierTestParams>;

/** @brief Test that runs a mesh modifier on a mesh and checks that a non-empty output mesh is created */
TEST_P(MeshModifierTestFixture, RunMeshModifiers)
{
  std::string mesh_file;
  std::shared_ptr<const MeshModifier> modifier;
  std::tie(mesh_file, modifier) = GetParam();

  pcl::PolygonMesh original_mesh;
  ASSERT_NO_THROW(original_mesh = loadMesh(std::filesystem::path(MESH_DIR) / mesh_file));

  std::vector<pcl::PolygonMesh> meshes;
  ASSERT_NO_THROW(meshes = modifier->modify(original_mesh));

  ASSERT_FALSE(meshes.empty());
  for (const pcl::PolygonMesh& mesh : meshes)
  {
    EXPECT_FALSE(mesh.polygons.empty());
    EXPECT_GT(mesh.cloud.width * mesh.cloud.height, 0);
  }
}

using DataRetainingMeshModifierTestFixture = testing::TestWithParam<MeshModifierTestParams>;

/** @brief Test that runs a mesh modifier on a mesh and checks that a non-empty output mesh is created and that the data
 * fields of the parent mesh are retained */
TEST_P(DataRetainingMeshModifierTestFixture, RunMeshModifiers)
{
  std::string mesh_file;
  std::shared_ptr<const MeshModifier> modifier;
  std::tie(mesh_file, modifier) = GetParam();

  pcl::PolygonMesh original_mesh;
  ASSERT_NO_THROW(original_mesh = loadMesh(std::filesystem::path(MESH_DIR) / mesh_file));

  std::vector<pcl::PolygonMesh> meshes;
  ASSERT_NO_THROW(meshes = modifier->modify(original_mesh));

  ASSERT_FALSE(meshes.empty());
  for (const pcl::PolygonMesh& mesh : meshes)
  {
    EXPECT_FALSE(mesh.polygons.empty());
    EXPECT_GT(mesh.cloud.width * mesh.cloud.height, 0);

    // Check that the output mesh fragment vertex cloud retains all the same data fields as the original mesh
    ASSERT_GE(mesh.cloud.fields.size(), original_mesh.cloud.fields.size());
    for (std::size_t i = 0; i < original_mesh.cloud.fields.size(); ++i)
    {
      ASSERT_TRUE(original_mesh.cloud.fields[i] == mesh.cloud.fields[i]);
    }
  }
}

/** @brief Creates a vector of implementations for the modifiers */
std::vector<std::shared_ptr<const MeshModifier>> createModifiers(const std::string& config_str)
{
  // Create the factory
  auto loader = std::make_shared<boost_plugin_loader::PluginLoader>();
  loader->search_libraries.emplace_back(NOETHER_PLUGIN_LIB);
  loader->search_paths.emplace_back(PLUGIN_DIR);
  Factory factory(loader);

  // Load the plugin YAML config
  YAML::Node config = YAML::Load(config_str);

  // Load the mesh modifiers
  std::vector<std::shared_ptr<const MeshModifier>> modifiers;
  modifiers.reserve(config.size());

  for (const YAML::Node& entry_config : config)
  {
    auto name = YAML::getMember<std::string>(entry_config, "name");
    modifiers.push_back(factory.createMeshModifier(entry_config));
  }

  return modifiers;
}

/** @brief Creates a list of mesh files (relative to `MESH_DIR`) to use in the unit test */
std::vector<std::string> getMeshFiles()
{
  return {
    "wavy_mesh_with_hole.ply",
  };
}

INSTANTIATE_TEST_SUITE_P(MeshModifierTests,
                         MeshModifierTestFixture,
                         testing::Combine(testing::ValuesIn(getMeshFiles()),
                                          testing::ValuesIn(createModifiers(all_mesh_modifiers))),
                         printMeshModifierTestParams);

INSTANTIATE_TEST_SUITE_P(MeshModifierTests,
                         DataRetainingMeshModifierTestFixture,
                         testing::Combine(testing::ValuesIn(getMeshFiles()),
                                          testing::ValuesIn(createModifiers(mesh_modifiers_data_retaining))),
                         printMeshModifierTestParams);

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
