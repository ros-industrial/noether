
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
  vtkSmartPointer<vtkPoints> empty;
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(empty);

  // Set input mesh
  path_planner::PathPlanner planner;
  planner.setInputMesh(data);

  // Set input tool data
  path_planner::ProcessTool tool;
  tool.pt_spacing = 0.5;
  tool.line_spacing = 1.0;
  tool.tool_offset = 0.0; // currently unused
  tool.intersecting_plane_height = 0.5; // 0.5 works best, not sure if this should be included in the tool
  tool.nearest_neighbors = 5; // not sure if this should be a part of the tool
  planner.setTool(tool);

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
  normals_data = planner.getInputMesh();
  vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
  viz.addPolyNormalsDisplay(normals_data, color, glyph);


  path_planner::ProcessPath path;
  planner.getFirstPath(path);
  planner.computePaths();
  std::vector<path_planner::ProcessPath> paths = planner.getPaths();

  for(int i = 0; i < paths.size(); ++i)
  {
    if(1) // display line
    {
      color[0] = 0.2;
      color[1] = 0.9;
      color[2] = 0.2;
      vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
      viz.addPolyNormalsDisplay(paths[i].line, color, glyph);
    }

    if(1) // display derivatives
    {
    color[0] = 0.9;
    color[1] = 0.9;
    color[2] = 0.2;
    vtkSmartPointer<vtkGlyph3D> glyph2 = vtkSmartPointer<vtkGlyph3D>::New();
    viz.addPolyNormalsDisplay(paths[i].derivatives, color, glyph2);
    }

    if(0) // Display cutting mesh
    {
      color[0] = 0.9;
      color[1] = 0.9;
      color[2] = 0.9;
      viz.addPolyDataDisplay(paths[i].intersection_plane, color);
    }
  }

  viz.renderDisplay();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
