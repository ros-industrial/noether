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
#include <noether_tpp/tool_path_planners/raster/direction_generators/fixed_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/fixed_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
// Edge planner
#include <noether_tpp/tool_path_planners/edge/edge_planner.h>
#include <noether_tpp/tool_path_planners/edge/boundary_edge_planner.h>
// Planner implementations
// Utilities
#include "utils.h"

using namespace noether;

/** @brief Create a simple planar square mesh of fixed size */
pcl::PolygonMesh createPlaneMesh(const float dim, const Eigen::Isometry3d& transform)
{
  pcl::PolygonMesh mesh;

  // Add 4 vertices "counter clockwise"
  pcl::PointCloud<pcl::PointNormal> cloud;

  {
    pcl::PointNormal p1;
    p1.getVector3fMap() = Eigen::Vector3f(0.0, 0.0, 0.0);
    p1.getNormalVector3fMap() = Eigen::Vector3f::UnitZ();
    cloud.push_back(p1);
  }
  {
    pcl::PointNormal p2;
    p2.getVector3fMap() = Eigen::Vector3f(dim, 0.0, 0.0);
    p2.getNormalVector3fMap() = Eigen::Vector3f::UnitZ();
    cloud.push_back(p2);
  }
  {
    pcl::PointNormal p3;
    p3.getVector3fMap() = Eigen::Vector3f(dim, dim, 0.0);
    p3.getNormalVector3fMap() = Eigen::Vector3f::UnitZ();
    cloud.push_back(p3);
  }
  {
    pcl::PointNormal p4;
    p4.getVector3fMap() = Eigen::Vector3f(0.0, dim, 0.0);
    p4.getNormalVector3fMap() = Eigen::Vector3f::UnitZ();
    cloud.push_back(p4);
  }

  // Apply the transform offset
  pcl::PointCloud<pcl::PointNormal> transformed_cloud;
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
  const unsigned n_lines{ 10 };
  const unsigned n_points{ 10 };
  const double min_hole_size{ 0.05 };
};

TEST_P(RasterPlannerTestFixture, FlatSquareMesh)
{
  // Create a flat plane mesh with 2 triangles
  double dim = 1.0;
  const Eigen::Isometry3d transform = createRandomTransform(dim * 5.0, M_PI);
  const Eigen::Vector3d direction = transform.rotation() * Eigen::Vector3d::UnitX();
  pcl::PolygonMesh mesh = createPlaneMesh(static_cast<float>(dim), transform);

  // Configure the planning factory to generate an arbitrary number of lines along the mesh x-axis starting at the mesh
  // origin
  std::shared_ptr<RasterPlannerFactory> factory = GetParam();
  factory->line_spacing = dim / static_cast<double>(n_lines);
  factory->point_spacing = dim / static_cast<double>(n_points + 1);
  factory->direction_gen = [&direction]() { return std::make_unique<FixedDirectionGenerator>(direction); };
  factory->origin_gen = [&transform]() { return std::make_unique<FixedOriginGenerator>(transform.translation()); };

  // Create the tool paths
  std::unique_ptr<const ToolPathPlanner> planner = factory->create();
  ToolPaths tool_paths;
  ASSERT_NO_THROW(tool_paths = planner->plan(mesh));

  EXPECT_EQ(tool_paths.size(), n_lines);
  for (ToolPath& path : tool_paths)
  {
    // Check that there is only a single segment in the path
    EXPECT_EQ(path.size(), 1);

    // Check the number of waypoints in the segment
    // TODO: Reconsider removing the point spacing from of the raster planner base class
    // EXPECT_EQ(path.at(0).size(), n_points);

    // Check that the segment aligns with the direction generator closely
    const Eigen::Isometry3d& first = path.front().front();
    const Eigen::Isometry3d& last = path.back().back();
    const Eigen::Vector3d dir = (last.translation() - first.translation()).normalized();
    EXPECT_GT(abs(dir.dot(direction)), std::cos(1.0 * M_PI / 180.0));
  }
}

TEST_P(RasterPlannerTestFixture, SemiPlanarMeshFile)
{
  // Load a mesh from the test directory
  pcl::PolygonMesh mesh = loadWavyMeshWithHole();
  const Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
  const double dim = 10.0;  // Square dimension of the mesh

  // Configure the planner factory
  std::shared_ptr<RasterPlannerFactory> factory = GetParam();
  factory->line_spacing = dim / (n_lines);
  factory->point_spacing = dim / (n_points + 1);
  factory->direction_gen = [&direction]() { return std::make_unique<FixedDirectionGenerator>(direction); };
  factory->origin_gen = [this]() { return std::make_unique<FixedOriginGenerator>(Eigen::Vector3d::Zero()); };

  // Plan
  std::unique_ptr<const ToolPathPlanner> planner = factory->create();
  ToolPaths tool_paths;
  ASSERT_NO_THROW(tool_paths = planner->plan(mesh));

  EXPECT_EQ(tool_paths.size(), n_lines);
  for (std::size_t i = 0; i < tool_paths.size(); ++i)
  {
    const ToolPath& path = tool_paths[i];

    // Check that each tool path has at least one segment
    EXPECT_GE(path.size(), 1);
    EXPECT_GE(path.front().size(), 2);

    // For tool paths over the hole in the mesh
    if (i > 3 && i < 7)
    {
      // Check that the paths that go over the hole get split into two
      EXPECT_EQ(path.size(), 2);
      // Check that the second path has at least two waypoints
      EXPECT_GE(path.back().size(), 2);
    }

    // Check that the segment aligns with the direction generator closely
    // The tolerance for this check should be somewhat high since the mesh is wavy and the first and last points could
    // be at different "heights"
    const Eigen::Isometry3d& first = path.front().front();
    const Eigen::Isometry3d& last = path.back().back();
    const Eigen::Vector3d dir = (last.translation() - first.translation()).normalized();
    EXPECT_GT(abs(dir.dot(direction)), std::cos(10.0 * M_PI / 180.0));
  }
}

/** @brief Returns a list of edge planner factory implementations */
std::vector<std::shared_ptr<RasterPlannerFactory>> createRasterPlannerFactories()
{
  auto plane_slicer_factory = std::make_shared<PlaneSlicerRasterPlannerFactory>();
  plane_slicer_factory->min_segment_size = 0.010;
  plane_slicer_factory->search_radius = 0.010;
  plane_slicer_factory->bidirectional = true;

  return {
    plane_slicer_factory,
  };
}

// Instantiate the implementations of the raster planner factory to test
INSTANTIATE_TEST_SUITE_P(RasterPlannerTests,
                         RasterPlannerTestFixture,
                         testing::ValuesIn(createRasterPlannerFactories()),
                         print<RasterPlannerFactory>);

using EdgePlannerTestFixture = testing::TestWithParam<std::shared_ptr<EdgePlannerFactory>>;

TEST_P(EdgePlannerTestFixture, FlatSquareMesh)
{
  // Create a flat plane mesh with 2 triangles
  double dim = 1.0;
  Eigen::Isometry3d transform = createRandomTransform(dim * 5.0, M_PI);
  pcl::PolygonMesh mesh = createPlaneMesh(static_cast<float>(dim), transform);

  // Configure the planning factory
  std::shared_ptr<EdgePlannerFactory> factory = GetParam();

  // Plan
  std::unique_ptr<const ToolPathPlanner> planner = factory->create();
  ToolPaths tool_paths;
  ASSERT_NO_THROW(tool_paths = planner->plan(mesh));

  // There should only be one edge path that goes around the border of the mesh
  EXPECT_EQ(tool_paths.size(), 1);
}

TEST_P(EdgePlannerTestFixture, SemiPlanarMeshFile)
{
  // Load the test mesh
  pcl::PolygonMesh mesh = loadWavyMeshWithHole();

  // Configure the planning factory
  std::shared_ptr<EdgePlannerFactory> factory = GetParam();

  // Plan
  std::unique_ptr<const ToolPathPlanner> planner = factory->create();
  ToolPaths tool_paths;
  ASSERT_NO_THROW(tool_paths = planner->plan(mesh));

  // There should only be two edge paths, one that goes around the border of the mesh and one that goes around the
  // border of the hole
  const std::size_t n_paths = 2;
  EXPECT_EQ(tool_paths.size(), n_paths);

  std::vector<double> path_lengths;
  path_lengths.reserve(n_paths);
  for (const ToolPath& path : tool_paths)
  {
    // Check that the path has at least one segment
    EXPECT_GE(path.size(), 1);

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
}

/** @brief Returns a list of edge planner factory implementations */
std::vector<std::shared_ptr<EdgePlannerFactory>> createEdgePlannerFactories()
{
  return {
    std::make_shared<BoundaryEdgePlannerFactory>(),
  };
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
