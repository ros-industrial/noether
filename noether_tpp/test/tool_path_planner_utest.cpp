#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <random>

// Planner interface
#include <noether_tpp/core/tool_path_planner.h>
// Raster planner
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators.h>
// Edge planner
#include <noether_tpp/tool_path_planners/edge/edge_planner.h>
// Planner implementations
// Utilities
#include "utils.h"

using namespace noether;

/** @brief Create a simple planar square mesh of fixed size */
pcl::PolygonMesh createPlaneMesh(const float dim, const Eigen::Isometry3d& transform)
{
  pcl::PolygonMesh mesh;

  // Add 4 vertices "counter clockwise"
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  cloud.push_back(pcl::PointXYZ(dim, 0.0, 0.0));
  cloud.push_back(pcl::PointXYZ(dim, dim, 0.0));
  cloud.push_back(pcl::PointXYZ(0.0, dim, 0.0));

  // Apply the transform offset
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud(cloud, transformed_cloud, transform.matrix());

  // Convert to message format
  pcl::toPCLPointCloud2(transformed_cloud, mesh.cloud);

  // Upper right triangle
  pcl::Vertices tri_1;
  tri_1.vertices = { 0, 1, 2 };
  mesh.polygons.push_back(tri_1);

  // Lower left triangle
  pcl::Vertices tri_2;
  tri_2.vertices = { 2, 3, 0 };
  mesh.polygons.push_back(tri_2);

  return mesh;
}

/**
 * @brief Loads a test mesh from the local test directory
 * @ details This mesh is a semi-planar sinusoidally-wavy square mesh of dimension (10, 10) with a hole of diameter 3
 * centered at (5, 5). The "bottom left" bound of the mesh is at (-1, -1)
 */
pcl::PolygonMesh loadWavyMeshWithHole()
{
  pcl::PolygonMesh mesh;
  const std::string mesh_file = MESH_DIR + std::string("/wavy_mesh_with_hole.ply");
  if (pcl::io::loadPolygonFile(mesh_file, mesh) > 0)
    return mesh;
  throw std::runtime_error("Failed to load test mesh from '" + mesh_file + "'");
}

/**
 * @brief Creates a random transform
 */
Eigen::Isometry3d createRandomTransform(const double translation_limit, const double rotation_limit)
{
  std::mt19937 rand_gen(0);

  // Create a random translation
  std::uniform_real_distribution<double> pos_dist(-std::abs(translation_limit), std::abs(translation_limit));
  Eigen::Vector3d pos = Eigen::Vector3d::NullaryExpr([&rand_gen, &pos_dist]() { return pos_dist(rand_gen); });

  // Create a random rotation
  std::uniform_real_distribution<double> rot_dist(-std::abs(rotation_limit), std::abs(rotation_limit));
  Eigen::Vector3d ea = Eigen::Vector3d::NullaryExpr([&rand_gen, &rot_dist]() { return rot_dist(rand_gen); });

  return Eigen::Translation3d(pos) * Eigen::AngleAxisd(ea(0), Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(ea(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(ea(2), Eigen::Vector3d::UnitZ());
}

/**
 * @brief Test fixture for all raster tool path planners
 */
class RasterPlannerTestFixture : public testing::TestWithParam<std::shared_ptr<RasterPlannerFactory>>
{
public:
  const unsigned n_lines{ 11 };
  const unsigned n_points{ 11 };
  const double min_hole_size{ 0.05 };
  const Eigen::Vector3d direction{ Eigen::Vector3d::UnitX() };
};

TEST_P(RasterPlannerTestFixture, FlatSquareMesh)
{
  // Create a flat plane mesh with 2 triangles
  double dim = 1.0;
  const Eigen::Isometry3d transform = createRandomTransform(dim * 5.0, M_PI);
  pcl::PolygonMesh mesh = createPlaneMesh(static_cast<float>(dim), transform);

  // Configure the planning factory to generate an arbitrary number of lines along the mesh x-axis starting at the mesh
  // origin
  std::shared_ptr<RasterPlannerFactory> factory = GetParam();
  factory->line_spacing = dim / static_cast<double>(n_lines - 1);
  factory->point_spacing = dim / static_cast<double>(n_points - 1);
  factory->direction_gen = [this, &transform]() {
    // Transform the nominal direction
    Eigen::Vector3d dir = transform * direction;
    return std::make_unique<FixedDirectionGenerator>(dir.normalized());
  };
  factory->origin_gen = []() { return std::make_unique<FixedOriginGenerator>(Eigen::Vector3d::Zero()); };

  // Create the tool paths
  std::unique_ptr<const ToolPathPlanner> planner = factory->create();
  ToolPaths tool_paths;
  ASSERT_NO_THROW(tool_paths = planner->plan(mesh));

  ASSERT_EQ(tool_paths.size(), n_lines);
  for (ToolPath& path : tool_paths)
  {
    // Check that there is only a single segment in the path
    ASSERT_EQ(path.size(), 1);
    // Check the number of waypoints in the segment
    ASSERT_EQ(path.at(0).size(), n_points);

    // Check that the segment aligns with the direction generator closely
    const Eigen::Isometry3d& first = path.front().front();
    const Eigen::Isometry3d& last = path.back().back();
    const Eigen::Vector3d dir = (first.inverse() * last).translation().normalized();
    ASSERT_GT(dir.dot(direction), std::cos(5 * M_PI / 180.0));
  }
}

TEST_P(RasterPlannerTestFixture, SemiPlanarMeshFile)
{
  // Load a mesh from the test directory
  pcl::PolygonMesh mesh = loadWavyMeshWithHole();
  const double dim = 10.0;  // Square dimension of the mesh

  // Configure the planner factory
  std::shared_ptr<RasterPlannerFactory> factory = GetParam();
  factory->line_spacing = dim / (n_lines - 1);
  factory->point_spacing = dim / (n_points - 1);
  factory->direction_gen = [this]() { return std::make_unique<FixedDirectionGenerator>(direction); };
  factory->origin_gen = [this]() { return std::make_unique<FixedOriginGenerator>(Eigen::Vector3d::Zero()); };

  // Plan
  std::unique_ptr<const ToolPathPlanner> planner = factory->create();
  ToolPaths tool_paths;
  ASSERT_NO_THROW(tool_paths = planner->plan(mesh));

  ASSERT_EQ(tool_paths.size(), n_lines);
  for (std::size_t i = 0; i < tool_paths.size(); ++i)
  {
    const ToolPath& path = tool_paths[i];

    // Check that each tool path has at least one segment
    ASSERT_GE(path.size(), 1);
    ASSERT_GE(path.front().size(), 2);

    // For tool paths over the hole in the mesh
    if (i > 3 && i < 7)
    {
      // Check that the paths that go over the hole get split into two
      ASSERT_EQ(path.size(), 2);
      // Check that the second path has at least two waypoints
      ASSERT_GE(path.back().size(), 2);
    }

    // Check that the segment aligns with the direction generator closely
    const Eigen::Isometry3d& first = path.front().front();
    const Eigen::Isometry3d& last = path.back().back();
    const Eigen::Vector3d dir = (first.inverse() * last).translation().normalized();
    ASSERT_GT(dir.dot(direction), std::cos(5 * M_PI / 180.0));
  }
}

/** @brief Returns a list of edge planner factory implementations */
std::vector<std::shared_ptr<RasterPlannerFactory>> createRasterPlannerFactories()
{
  std::vector<std::shared_ptr<RasterPlannerFactory>> v;
  return v;
}

// Instantiate the implementations of the raster planner factory to test
INSTANTIATE_TEST_SUITE_P(RasterPlannerTests,
                         RasterPlannerTestFixture,
                         testing::ValuesIn(createRasterPlannerFactories()),
                         print<RasterPlannerFactory>);

class EdgePlannerTestFixture : public testing::TestWithParam<std::shared_ptr<EdgePlannerFactory>>
{
public:
  const unsigned n_points{ 11 };
};

TEST_P(EdgePlannerTestFixture, FlatSquareMesh)
{
  // Create a flat plane mesh with 2 triangles
  double dim = 1.0;
  Eigen::Isometry3d transform = createRandomTransform(dim * 5.0, M_PI);
  pcl::PolygonMesh mesh = createPlaneMesh(static_cast<float>(dim), transform);

  // Configure the planning factory
  std::shared_ptr<EdgePlannerFactory> factory = GetParam();
  factory->point_spacing = dim / (n_points - 1);

  // Plan
  std::unique_ptr<const ToolPathPlanner> planner = factory->create();
  ToolPaths tool_paths;
  ASSERT_NO_THROW(tool_paths = planner->plan(mesh));

  // There should only be one edge path that goes around the border of the mesh
  ASSERT_EQ(tool_paths.size(), 1);
  ASSERT_EQ(tool_paths.front().size(), n_points);
}

TEST_P(EdgePlannerTestFixture, SemiPlanarMeshFile)
{
  // Load the test mesh
  pcl::PolygonMesh mesh = loadWavyMeshWithHole();
  const double dim = 10.0;  // The square dimension of the mesh

  // Configure the planning factory
  std::shared_ptr<EdgePlannerFactory> factory = GetParam();
  factory->point_spacing = dim / (n_points - 1);

  // Plan
  std::unique_ptr<const ToolPathPlanner> planner = factory->create();
  ToolPaths tool_paths;
  ASSERT_NO_THROW(tool_paths = planner->plan(mesh));

  // There should only be two edge paths, one that goes around the border of the mesh and one that goes around the
  // border of the hole
  const std::size_t n_paths = 2;
  ASSERT_EQ(tool_paths.size(), n_paths);

  std::vector<double> path_lengths;
  path_lengths.reserve(n_paths);
  for (const ToolPath& path : tool_paths)
  {
    // Check that the path has at least one segment
    ASSERT_GE(path.size(), 1);

    // Compute the length of the path
    double len = 0.0;
    for (const ToolPathSegment& segment : path)
    {
      for (std::size_t i = 0; i < path.size() - 1; ++i)
      {
        const Eigen::Isometry3d& first = path.front()[i];
        const Eigen::Isometry3d& second = path.front()[i + 1];
        len += (second.translation() - first.translation()).norm();
      }
    }
    path_lengths.push_back(len);
  }

  // Create a vector of indices representing the order in which the tool paths should have been sorted
  std::vector<std::size_t> expected_order(n_paths);
  std::iota(expected_order.begin(), expected_order.end(), 0);

  // Argsort the paths in descending order by length
  std::vector<std::size_t> sorted_order(expected_order);
  std::sort(sorted_order.begin(), sorted_order.end(), [&path_lengths](const std::size_t& a, const std::size_t& b) {
    return path_lengths.at(a) > path_lengths.at(b);
  });

  // Ensure the paths are ordered correctly by length
  ASSERT_TRUE(std::equal(expected_order.begin(), expected_order.end(), sorted_order.begin()));
}

/** @brief Returns a list of edge planner factory implementations */
std::vector<std::shared_ptr<EdgePlannerFactory>> createEdgePlannerFactories()
{
  std::vector<std::shared_ptr<EdgePlannerFactory>> v;
  return v;
}

// Instantiate the implementations of the edge planner factories to test
INSTANTIATE_TEST_SUITE_P(EdgePlannerTests,
                         EdgePlannerTestFixture,
                         testing::ValuesIn(createEdgePlannerFactories()),
                         print<EdgePlannerFactory>);

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
