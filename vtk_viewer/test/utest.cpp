/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <vtk_viewer/vtk_utils.h>
#include <gtest/gtest.h>


TEST(ViewerTest, TestCase1)
{
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane();
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(points);
  vtk_viewer::visualizePlane(data);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
