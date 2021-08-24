#include <boost/core/demangle.hpp>
#include <gtest/gtest.h>
#include <regex>

#include <noether_tpp/core/tool_path_modifier.h>
// Implementations
#include <noether_tpp/tool_path_modifiers/no_op_modifier.h>
#include <noether_tpp/tool_path_modifiers/waypoint_orientation_modifiers.h>

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

ToolPaths
createArbitraryToolPath(unsigned p, unsigned s, unsigned w, std::function<Eigen::Isometry3d()> waypoint_generator)
{
  ToolPaths paths(p);
  for (unsigned i = 0; i < p; ++i)
  {
    for (ToolPath& path : paths)
    {
      path.resize(s);
      for (ToolPathSegment& segment : path)
      {
        segment.resize(w);
        std::fill(segment.begin(), segment.end(), waypoint_generator());
      }
    }
  }
  return paths;
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
           std::make_shared<DirectionOfTravelOrientationModifier>() };
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

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
