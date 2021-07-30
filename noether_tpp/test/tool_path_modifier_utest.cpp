#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/tool_path_modifiers/no_op_modifier.h>

#include <boost/core/demangle.hpp>
#include <gtest/gtest.h>
#include <regex>

using namespace noether;

ToolPaths createArbitraryToolPath(unsigned p, unsigned s, unsigned w)
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
        std::fill(segment.begin(), segment.end(), Eigen::Isometry3d::Identity());
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
  ASSERT_NO_THROW(modifier->modify(createArbitraryToolPath(4, 1, 10)));
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

  // Create an arbitrary tool path
  ToolPaths input = createArbitraryToolPath(4, 1, 10);

  // Apply the modifier
  ToolPaths output_1;
  ASSERT_NO_THROW(output_1 = modifier->modify(input));

  // Apply the modifier again to the previous output and check that it does not change
  ToolPaths output_2 = modifier->modify(output_1);
  compare(output_1, output_2);
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
std::string print(testing::TestParamInfo<std::shared_ptr<const ToolPathModifier>> info)
{
  return getClassName(*info.param) + "_" + std::to_string(info.index);
}

/** @brief Prints name of class for one-time modifier specific unit test output */
std::string print2(testing::TestParamInfo<std::shared_ptr<const OneTimeToolPathModifier>> info)
{
  return getClassName(*info.param) + "_" + std::to_string(info.index);
}

// Define functions for creating implementations of various modifiers
std::shared_ptr<OneTimeToolPathModifier> create() { return std::make_shared<NoOpToolPathModifier>(); }

INSTANTIATE_TEST_SUITE_P(ToolPathModifierTests, ToolPathModifierTestFixture, testing::Values(create()), print);
INSTANTIATE_TEST_SUITE_P(OneTimeToolPathModifierTests,
                         OneTimeToolPathModifierTestFixture,
                         testing::Values(create()),
                         print2);

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
