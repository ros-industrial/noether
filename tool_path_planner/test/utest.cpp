
/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <tool_path_planner/tool_path_planner.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <gtest/gtest.h>
#include <vtkIdTypeArray.h>

#define DISPLAY_LINES  1
#define DISPLAY_NORMALS  0
#define DISPLAY_DERIVATIVES  1
#define DISPLAY_CUTTING_MESHES  0


TEST(IntersectTest, TestCase1)
{
  // Get mesh
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane();
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(points, 0.5, 5);
  vtk_viewer::generateNormals(data);

  // Set input mesh
  tool_path_planner::ToolPathPlanner planner;
  planner.setInputMesh(data);

  // Set input tool data
  tool_path_planner::ProcessTool tool;
  tool.pt_spacing = 0.2;
  tool.line_spacing = 0.5;
  tool.tool_offset = 0.0; // currently unused
  tool.intersecting_plane_height = 0.1; // 0.5 works best, not sure if this should be included in the tool
  tool.nearest_neighbors = 30; // not sure if this should be a part of the tool
  planner.setTool(tool);

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 1.0;

  // Display mesh results
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;
  viz.addPolyDataDisplay(data, color);


  // Display surface normals
  if(DISPLAY_NORMALS)
  {
    color[0] = 0.9;
    color[1] = 0.1;
    color[2] = 0.1;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInputMesh();
    vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
    viz.addPolyNormalsDisplay(normals_data, color, glyph, scale);
  }


  // Plan paths for given mesh
  tool_path_planner::ProcessPath path;
  planner.getFirstPath(path);
  planner.computePaths();
  std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();

  for(int i = 0; i < paths.size(); ++i)
  {
    if(DISPLAY_LINES) // display line
    {
      color[0] = 0.2;
      color[1] = 0.9;
      color[2] = 0.2;
      vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
      viz.addPolyNormalsDisplay(paths[i].line, color, glyph, scale);
    }

    if(DISPLAY_DERIVATIVES) // display derivatives
    {
    color[0] = 0.9;
    color[1] = 0.9;
    color[2] = 0.2;
    vtkSmartPointer<vtkGlyph3D> glyph2 = vtkSmartPointer<vtkGlyph3D>::New();
    viz.addPolyNormalsDisplay(paths[i].derivatives, color, glyph2, scale);
    }

    if(DISPLAY_CUTTING_MESHES) // Display cutting mesh
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
