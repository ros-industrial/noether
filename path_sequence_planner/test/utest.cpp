

/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <path_sequence_planner/path_sequence_planner.h>
#include <tool_path_planner/tool_path_planner.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <gtest/gtest.h>
#include <vtkIdTypeArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>

#define DISPLAY_LINES  1
#define DISPLAY_NORMALS  1
#define DISPLAY_DERIVATIVES  1
#define DISPLAY_CUTTING_MESHES  0


TEST(IntersectTest, TestCase1)
{
  // Get mesh
  vtkSmartPointer<vtkPoints> empty;
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(empty);
  vtk_viewer::generateNormals(data);

  // Set input mesh
  tool_path_planner::ToolPathPlanner planner;
  planner.setInputMesh(data);

  // Set input tool data
  tool_path_planner::ProcessTool tool;
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
  if(DISPLAY_NORMALS)
  {
    color[0] = 0.9;
    color[1] = 0.1;
    color[2] = 0.1;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInputMesh();
    vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
    viz.addPolyNormalsDisplay(normals_data, color, glyph);
  }

  tool_path_planner::ProcessPath path;
  planner.getFirstPath(path);
  planner.computePaths();
  std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();


  // Create sequence planner and set the data
  path_sequence_planner::PathSequencePlanner sequence_planner;
  sequence_planner.setPaths(paths);

  sequence_planner.linkPaths();

  std::vector<tool_path_planner::ProcessPath> paths2 = sequence_planner.getPaths();
  std::vector<int> indices = sequence_planner.getIndices();

  vtkSmartPointer<vtkPolyData> connecting_data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> connecting_points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkDoubleArray> normals = vtkSmartPointer<vtkDoubleArray>::New();
  normals->SetNumberOfComponents(3);
  for(int i = 0; i < paths2.size(); ++i)
  {
    if(DISPLAY_LINES) // display line
    {
      color[0] = 0.2;
      color[1] = 0.9;
      color[2] = 0.2;
      vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
      viz.addPolyNormalsDisplay(paths2[i].line, color, glyph);
    }

    if(DISPLAY_DERIVATIVES) // display derivatives
    {
    color[0] = 0.9;
    color[1] = 0.9;
    color[2] = 0.2;
    vtkSmartPointer<vtkGlyph3D> glyph2 = vtkSmartPointer<vtkGlyph3D>::New();
    viz.addPolyNormalsDisplay(paths2[i].derivatives, color, glyph2);
    }

    if(DISPLAY_CUTTING_MESHES) // Display cutting mesh
    {
      color[0] = 0.9;
      color[1] = 0.9;
      color[2] = 0.9;
      viz.addPolyDataDisplay(paths2[i].intersection_plane, color);
    }

    if(i > 0)
    {
      double* pt1 = paths2[i-1].line->GetPoints()->GetPoint(paths2[i-1].line->GetPoints()->GetNumberOfPoints()-1);
      double* pt2 = paths2[i].line->GetPoints()->GetPoint(0);
      connecting_points->InsertNextPoint(pt2);
      double norm[3];
      norm[0] = pt1[0] - pt2[0];
      norm[1] = pt1[1] - pt2[1];
      norm[2] = pt1[2] - pt2[2];
      normals->InsertNextTuple(norm);
    }
  }
  //connecting_points->SetNormals(normals);
  connecting_data->SetPoints(connecting_points);
  connecting_data->GetPointData()->SetNormals(normals);

  color[0] = 0.2;
  color[1] = 0.9;
  color[2] = 0.9;
  vtkSmartPointer<vtkGlyph3D> glyph3 = vtkSmartPointer<vtkGlyph3D>::New();
  viz.addPolyNormalsDisplay(connecting_data, color, glyph3);

  viz.renderDisplay();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
