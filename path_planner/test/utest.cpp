
/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <path_planner/path_planner.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <gtest/gtest.h>
#include <vtkIdTypeArray.h>

using namespace vtk_viewer;
using namespace path_planner;

TEST(IntersectTest, TestCase1)
{
  // Get mesh
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh();

  // Calculate intersection
  path_planner::PathPlanner planner;
  planner.setInputMesh(data);

  vtkSmartPointer<vtkPolyData> intersect_data;
  intersect_data = planner.getFirstPath();

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  // Display mesh results
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;
  viz.addPolyDataDisplay(data, color);


  // Display surface normals
  color[0] = 0.9;
  color[1] = 0.1;
  color[2] = 0.1;
  vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
  normals_data = planner.generateNormals(data);
  vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
  viz.addPolyNormalsDisplay(normals_data, color, glyph);

  // Display line intersection
  color[0] = 0.7;
  color[1] = 0.2;
  color[2] = 0.2;
  viz.addPolyDataDisplay(intersect_data, color);

  // Smooth the intersection points and display evenly spaced points
  color[0] = 0.2;
  color[1] = 0.2;
  color[2] = 0.9;
  vtkSmartPointer<vtkPolyData> smooth_data;
  vtkSmartPointer<vtkPoints> points;
  points = intersect_data->GetPoints();
  smooth_data = planner.smoothData(points);

  // display smooth line
  color[0] = 0.2;
  color[1] = 0.9;
  color[2] = 0.9;
  viz.addPolyDataDisplay(smooth_data, color);

  // display evenly spaced points
  vtkSmartPointer<vtkPoints> points2;
  points2 = smooth_data->GetPoints();
  viz.addPointDataDisplay(points2, color);

  viz.renderDisplay();
  //vtk_viewer::visualizePlane(data);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
