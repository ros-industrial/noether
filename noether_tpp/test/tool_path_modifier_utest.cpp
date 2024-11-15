#include <gtest/gtest.h>
#include <random>

#include <noether_tpp/core/tool_path_modifier.h>
// Implementations
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
#include <noether_tpp/tool_path_modifiers/uniform_spacing_spline_modifier.h>
#include <noether_tpp/tool_path_modifiers/standard_edge_paths_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/tool_drag_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/uniform_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/uniform_spacing_linear_modifier.h>
#include "utils.h"

using namespace noether;

/** @brief Interface for creating waypoints for test tool paths */
struct WaypointCreator
{
  virtual Eigen::Isometry3d operator()() const = 0;
};

/** @brief Creates an identity waypoint */
struct IdentityWaypointCreator : WaypointCreator
{
  Eigen::Isometry3d operator()() const override final { return Eigen::Isometry3d::Identity(); }
};

/** @brief Creates a waypoint with random position and orientation */
struct RandomWaypointCreator : WaypointCreator
{
  RandomWaypointCreator() : gen(0), dist(-1.0, 1.0) {}

  Eigen::Isometry3d operator()() const override final
  {
    Eigen::Vector3d t = Eigen::Vector3d::NullaryExpr([this]() { return dist(gen); });
    Eigen::Vector3d r = Eigen::Vector3d::NullaryExpr([this]() { return dist(gen) * EIGEN_PI; });
    Eigen::Isometry3d pose = Eigen::AngleAxisd(r.x(), Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(r.y(), Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(r.z(), Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(t);
    return pose;
  }

  mutable std::mt19937 gen;
  mutable std::uniform_real_distribution<double> dist;
};

ToolPaths
createArbitraryToolPath(const unsigned p, const unsigned s, const unsigned w, const WaypointCreator& waypoint_creator)
{
  ToolPaths paths(p);
  for (ToolPath& path : paths)
  {
    path.resize(s);
    for (ToolPathSegment& segment : path)
    {
      segment.resize(w);
      std::generate(segment.begin(), segment.end(), std::bind(&WaypointCreator::operator(), &waypoint_creator));
    }
  }

  return paths;
}

ToolPaths createRasterGridToolPath(const unsigned p, const unsigned s, const unsigned w)
{
  ToolPaths paths;
  paths.reserve(p);
  for (unsigned p_idx = 0; p_idx < p; ++p_idx)
  {
    ToolPath path;
    path.reserve(s);

    for (unsigned s_idx = 0; s_idx < s; ++s_idx)
    {
      ToolPathSegment segment;
      segment.reserve(w);

      for (unsigned w_idx = 0; w_idx < w; ++w_idx)
      {
        Eigen::Isometry3d waypoint(Eigen::Isometry3d::Identity());
        waypoint.translation().x() = static_cast<double>(s_idx * w + w_idx);
        waypoint.translation().y() = static_cast<double>(p_idx);
        waypoint.translation().z() = 0.0;
        segment.push_back(waypoint);
      }

      path.push_back(segment);
    }

    paths.push_back(path);
  }

  return paths;
}

ToolPaths createSquareToolPaths(const unsigned p, const unsigned w)
{
  // Helper function for creating a segment given a start position and offset direction
  auto create_segment = [&w](const Eigen::Isometry3d& start, const Eigen::Vector3d& offset) -> ToolPathSegment {
    ToolPathSegment segment;
    segment.reserve(w);

    for (unsigned w_idx = 0; w_idx < w; ++w_idx)
    {
      segment.push_back(start * Eigen::Translation3d(offset * w_idx));
    }

    return segment;
  };

  ToolPaths paths;
  paths.reserve(p);

  // Vector of scales such that each tool path will be a different sized square
  Eigen::VectorXd scales = Eigen::VectorXd::LinSpaced(Eigen::Sequential, p, 1.0, 2.0).reverse();

  for (unsigned p_idx = 0; p_idx < p; ++p_idx)
  {
    ToolPath path;
    path.reserve(4);

    // Create a square of tool path segments going clockwise from the top-left corner
    Eigen::Vector3d offset(scales[p_idx], 0.0, 0.0);
    // Top
    path.push_back(create_segment(Eigen::Isometry3d::Identity(), offset));
    // Right
    path.push_back(create_segment(path.back().back() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()), offset));
    // Bottom
    path.push_back(create_segment(path.back().back() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()), offset));
    // Left
    path.push_back(create_segment(path.back().back() * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()), offset));

    paths.push_back(path);
  }

  return paths;
}

ToolPaths shuffle(ToolPaths tool_paths, const bool shuffle_waypoints, const std::size_t seed = 0)
{
  // Seeded random number generator
  std::mt19937 rand(seed);

  for (ToolPath& tool_path : tool_paths)
  {
    if (shuffle_waypoints)
    {
      for (ToolPathSegment& segment : tool_path)
      {
        // Shuffle the order of waypoints in tool path segments
        std::shuffle(segment.begin(), segment.end(), rand);
      }
    }

    // Shuffle the order of tool path segments in the tool path
    std::shuffle(tool_path.begin(), tool_path.end(), rand);
  }

  // Shuffle the order of tool paths 1 through n within the container. Keep the original first tool path first
  std::shuffle(tool_paths.begin() + 1, tool_paths.end(), rand);

  return tool_paths;
}

/**
 * @brief Compares two tool paths for approximate equality
 * @details Eigen::Transform<...> doesn't provide an equality operator, so we need to write a custom function to compare
 * two tool paths for approximate equality
 */
void compare(const ToolPaths& a, const ToolPaths& b)
{
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    ASSERT_EQ(a[i].size(), b[i].size());
    for (std::size_t j = 0; j < a[i].size(); ++j)
    {
      ASSERT_EQ(a[i][j].size(), b[i][j].size());
      for (std::size_t k = 0; k < a[i][j].size(); ++k)
      {
        ASSERT_TRUE(a[i][j][k].isApprox(b[i][j][k])) << "Path " << i << ", Segment " << j << ", Waypoint " << k;
      }
    }
  }
}

/**
 * @brief Test fixture for all tool path modifiers
 */
class ToolPathModifierTestFixture : public testing::TestWithParam<std::shared_ptr<const ToolPathModifier>>
{
};

TEST_P(ToolPathModifierTestFixture, TestOperation)
{
  auto modifier = GetParam();
  ASSERT_NO_THROW(modifier->modify(createArbitraryToolPath(4, 2, 10, IdentityWaypointCreator())));
  ASSERT_NO_THROW(modifier->modify(createArbitraryToolPath(4, 2, 10, RandomWaypointCreator())));
  ASSERT_NO_THROW(modifier->modify(createRasterGridToolPath(4, 2, 10)));
  ASSERT_NO_THROW(modifier->modify(createSquareToolPaths(4, 10)));
}

/**
 * @brief Test fixture specifically for one-time tool path modifiers
 */
class OneTimeToolPathModifierTestFixture : public testing::TestWithParam<std::shared_ptr<const OneTimeToolPathModifier>>
{
};

TEST_P(OneTimeToolPathModifierTestFixture, TestOperation)
{
  auto modifier = GetParam();

  // Create several arbitrary tool paths
  std::vector<ToolPaths> inputs = { createArbitraryToolPath(4, 2, 10, IdentityWaypointCreator()),
                                    createArbitraryToolPath(4, 2, 10, RandomWaypointCreator()),
                                    createRasterGridToolPath(4, 2, 10),
                                    createSquareToolPaths(4, 10) };

  for (const ToolPaths& input : inputs)
  {
    // Apply the modifier
    ToolPaths output_1;
    ASSERT_NO_THROW(output_1 = modifier->modify(input));

    // Apply the modifier again to the previous output and check that it does not change
    ToolPaths output_2 = modifier->modify(output_1);
    compare(output_1, output_2);
  }
}

// Create a vector of implementations for the modifiers
std::vector<std::shared_ptr<const ToolPathModifier>> createModifiers()
{
  return { std::make_shared<BiasedToolDragOrientationToolPathModifier>(10 * M_PI / 180.0, 0.025),
           std::make_shared<CircularLeadInModifier>(M_PI / 2.0, 0.1, 5),
           std::make_shared<CircularLeadOutModifier>(M_PI / 2.0, 0.1, 5),
           std::make_shared<ConcatenateModifier>(),
           std::make_shared<DirectionOfTravelOrientationModifier>(),
           std::make_shared<FixedOrientationModifier>(Eigen::Vector3d(1, 0, 0)),
           std::make_shared<LinearApproachModifier>(Eigen::Vector3d(0.1, 0.2, 0.3), 5),
           std::make_shared<LinearApproachModifier>(0.1, LinearApproachModifier::Axis::Z, 5),
           std::make_shared<LinearDepartureModifier>(Eigen::Vector3d(0.1, 0.2, 0.3), 5),
           std::make_shared<LinearDepartureModifier>(0.1, LinearDepartureModifier::Axis::Z, 5),
           std::make_shared<MovingAverageOrientationSmoothingModifier>(3),
           std::make_shared<OffsetModifier>(Eigen::Isometry3d::Identity()),
           std::make_shared<RasterOrganizationModifier>(),
           std::make_shared<SnakeOrganizationModifier>(),
           std::make_shared<UniformSpacingSplineModifier>(0.025),
           std::make_shared<StandardEdgePathsOrganizationModifier>(),
           std::make_shared<ToolDragOrientationToolPathModifier>(10 * M_PI / 180.0, 0.025),
           std::make_shared<UniformOrientationModifier>(),
           std::make_shared<UniformSpacingLinearModifier>(0.025) };
}

std::vector<std::shared_ptr<const OneTimeToolPathModifier>> createOneTimeModifiers()
{
  std::vector<std::shared_ptr<const ToolPathModifier>> modifiers = createModifiers();
  std::vector<std::shared_ptr<const OneTimeToolPathModifier>> one_time_modifiers;
  one_time_modifiers.reserve(modifiers.size());
  for (auto modifier : modifiers)
  {
    auto m = std::dynamic_pointer_cast<const OneTimeToolPathModifier>(modifier);
    if (m != nullptr)
      one_time_modifiers.push_back(m);
  }
  return one_time_modifiers;
}

INSTANTIATE_TEST_SUITE_P(ToolPathModifierTests,
                         ToolPathModifierTestFixture,
                         testing::ValuesIn(createModifiers()),
                         print<const ToolPathModifier>);

INSTANTIATE_TEST_SUITE_P(OneTimeToolPathModifierTests,
                         OneTimeToolPathModifierTestFixture,
                         testing::ValuesIn(createOneTimeModifiers()),
                         print<const OneTimeToolPathModifier>);

///////////////////////////////////
// Implementation-specific tests //
///////////////////////////////////

TEST(ToolPathModifierTests, OrganizationModifiersTest)
{
  // Create a raster grid tool path pattern
  const unsigned n_paths = 4;
  const unsigned n_segments = 2;
  const unsigned n_waypoints = 10;
  const ToolPaths tool_paths = createRasterGridToolPath(n_paths, n_segments, n_waypoints);
  const ToolPaths shuffled_tool_paths = shuffle(tool_paths, true);

  // Create a raster pattern from the original tool paths
  RasterOrganizationModifier raster;
  ToolPaths raster_tool_paths = raster.modify(tool_paths);

  // The original tool path is already a raster, so a raster modifier shouldn't change the structure
  compare(tool_paths, raster_tool_paths);

  // Modify the shuffled tool path to produce a raster pattern
  raster_tool_paths = raster.modify(shuffled_tool_paths);
  compare(tool_paths, raster_tool_paths);

  // Create a snake pattern from the raster tool paths
  SnakeOrganizationModifier snake;
  const ToolPaths snake_tool_paths = snake.modify(raster_tool_paths);

  // Check the snake pattern
  for (unsigned i = 0; i < n_paths - 1; ++i)
  {
    const ToolPath& first = snake_tool_paths.at(i);
    const ToolPath& second = snake_tool_paths.at(i + 1);

    // The last waypoint in the last segment of the first path should have the same x-axis value as the first waypoint
    // in the first segment of the second path
    ASSERT_DOUBLE_EQ(first.back().back().translation().x(), second.front().front().translation().x());
  }

  // Convert the snake tool paths into raster tool paths and compare to the original
  raster_tool_paths = raster.modify(snake_tool_paths);
  compare(tool_paths, raster_tool_paths);
}

TEST(ToolPathModifierTests, EdgePathOrganizationModifiersTest)
{
  // Create a set of square tool paths to represent a collection of edge paths
  const unsigned n_paths = 4;
  const unsigned n_waypoints = 10;
  const ToolPaths tool_paths = createSquareToolPaths(n_paths, n_waypoints);

  // Create an ordered edge path organization modifier
  StandardEdgePathsOrganizationModifier edge_path_organizer;

  // The original square tool path should already match the output of the ordered edge path organization modifier
  compare(edge_path_organizer.modify(tool_paths), tool_paths);

  // Shuffle the tool paths and re-sort them into an ordered set of edge paths
  // Note: the standard edge path organization modifier expects the planner to have provided the waypoints in the
  // correct order, so don't shuffle them within the segments/tool paths
  ToolPaths edge_tool_paths = edge_path_organizer.modify(shuffle(tool_paths, false));

  // Check for equivalence with the original tool path
  compare(edge_tool_paths, tool_paths);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
