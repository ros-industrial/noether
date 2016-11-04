/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <vtk_viewer/vtk_utils.h>
#include <gtest/gtest.h>

using namespace vtk_viewer;

TEST(ViewerTest, TestCase1)
{
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh();
  vtk_viewer::visualizePlane(data);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
