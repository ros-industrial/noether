#include <boost/core/demangle.hpp>
#include <gtest/gtest.h>
#include <random>
#include <regex>

#include <noether_tpp/core/tool_path_modifier.h>
// Implementations
#include <noether_tpp/tool_path_modifiers/no_op_modifier.h>
#include <noether_tpp/tool_path_modifiers/waypoint_orientation_modifiers.h>
#include <noether_tpp/tool_path_modifiers/organization_modifiers.h>

using namespace noether;

/** @brief Creates an identity waypoint */
Eigen::Isometry3d createIdentityWaypoint() { return Eigen::Isometry3d::Identity(); }

/** @brief Creates a waypoint with random position and orientation */
Eigen::Isometry3d createRandomWaypoint()
{
  Eigen::Vector3d t = Eigen::Vector3d::Random();
  Eigen::Vector3d r = Eigen::Vector3d::Random() * EIGEN_PI;
  Eigen::Isometry3d pose = Eigen::AngleAxisd(r.x(), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(r.y(), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(r.z(), Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(t);
  return pose;
}

ToolPaths createArbitraryToolPath(const unsigned p,
                                  const unsigned s,
                                  const unsigned w,
                                  std::function<Eigen::Isometry3d()> waypoint_generator)
{
  ToolPaths paths(p);
  for (ToolPath& path : paths)
  {
    path.resize(s);
    for (ToolPathSegment& segment : path)
    {
      segment.resize(w);
      std::fill(segment.begin(), segment.end(), waypoint_generator());
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

ToolPaths shuffle(ToolPaths tool_paths, std::size_t seed = 0)
{
  // Seeded random number generator
  std::mt19937 rand(seed);

  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      // Shuffle the order of waypoints in tool path segments
      std::shuffle(segment.begin(), segment.end(), rand);
    }

    // Shuffle the order of tool path segments in the tool path
    std::shuffle(tool_path.begin(), tool_path.end(), rand);
  }

  // Shuffle the order of tool paths within the container
  std::shuffle(tool_paths.begin(), tool_paths.end(), rand);

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
        ASSERT_TRUE(a[i][j][k].isApprox(b[i][j][k]));
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
  ASSERT_NO_THROW(modifier->modify(createArbitraryToolPath(4, 1, 10, createIdentityWaypoint)));
  ASSERT_NO_THROW(modifier->modify(createArbitraryToolPath(4, 1, 10, createRandomWaypoint)));
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
  std::vector<ToolPaths> inputs = { createArbitraryToolPath(4, 1, 10, createIdentityWaypoint),
                                    createArbitraryToolPath(4, 1, 10, createRandomWaypoint) };

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

/** @brief Extracts the demangled class name behind the namespace for printing in unit test */
std::string getClassName(const ToolPathModifier& modifier)
{
  std::regex re(".*::(.*)");
  std::smatch match;
  std::string class_name = boost::core::demangle(typeid(modifier).name());
  if (std::regex_match(class_name, match, re))
    return match[1];
  throw std::runtime_error("Failed to get class name from demangled name");
}

/** @brief Prints name of class for unit test output */
template <typename T>
std::string print(testing::TestParamInfo<std::shared_ptr<const T>> info)
{
  return getClassName(*info.param) + "_" + std::to_string(info.index);
}

// Create a vector of implementations for the modifiers
std::vector<std::shared_ptr<const ToolPathModifier>> createModifiers()
{
  return { std::make_shared<NoOpToolPathModifier>(),
           std::make_shared<FixedOrientationModifier>(Eigen::Vector3d(1, 0, 0)),
           std::make_shared<UniformOrientationModifier>(),
           std::make_shared<DirectionOfTravelOrientationModifier>(),
           std::make_shared<RasterOrganizationModifier>(),
           std::make_shared<SnakeOrganizationModifier>() };
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
                         print<ToolPathModifier>);

INSTANTIATE_TEST_SUITE_P(OneTimeToolPathModifierTests,
                         OneTimeToolPathModifierTestFixture,
                         testing::ValuesIn(createOneTimeModifiers()),
                         print<OneTimeToolPathModifier>);

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
  const ToolPaths shuffled_tool_paths = shuffle(tool_paths);

  // Create a raster pattern from the original tool paths
  RasterOrganizationModifier raster;
  ToolPaths raster_tool_paths = raster.modify(tool_paths);

  // The original tool path is already a raster, so a raster modifier shouldn't change the structure
  compare(tool_paths, raster_tool_paths);

  // Modify the shuffled tool path to produce a raster pattern
  raster_tool_paths = raster.modify(shuffled_tool_paths);
  compare(tool_paths, raster_tool_paths);

  // Create a snake pattern from the shuffled tool paths
  SnakeOrganizationModifier snake;
  const ToolPaths snake_tool_paths = snake.modify(shuffled_tool_paths);

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

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
