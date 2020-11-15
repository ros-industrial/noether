#include "noether_conversions/noether_conversions.h"
#include <gtest/gtest.h>

TEST(TestSuite, GeometryMsgsToVTK_setup) { ASSERT_NO_FATAL_FAILURE(); }

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
