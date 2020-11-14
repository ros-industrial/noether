

/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <path_sequence_planner/simple_path_sequence_planner.h>
#include <tool_path_planner/surface_walk_raster_generator.h>
#include <tool_path_planner/utilities.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <gtest/gtest.h>
#include <vtkIdTypeArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>

#define DISPLAY_LINES 1
#define DISPLAY_NORMALS 0
#define DISPLAY_DERIVATIVES 1
#define DISPLAY_CUTTING_MESHES 0

// This test displays the results of the path sequencer and should look very similar to the results
// from the tool_path_planner unit test.  This test adds cyan arrows which link the paths together
// into one continous path.  Yellow arrows are flipped as necessary to maintain the contious nature of
// the path.  The output should look like a series of yellow paths, whose direction is alternating
// direction from up->down to down->up, linked by cyan arrows.

TEST(IntersectTest, TestCase1)
{
  // Get mesh
  vtkSmartPointer<vtkPoints> empty;
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(empty, 0.5, 5);
  vtk_viewer::generateNormals(data);

  // create cutout in the middle of the mesh
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  double pt1[3] = { 2.0, 3.0, 0.0 };
  double pt2[3] = { 4.0, 2.0, 0.0 };
  double pt3[3] = { 5.0, 3.0, 0.0 };
  double pt4[3] = { 4.0, 5.0, 0.0 };
  points2->InsertNextPoint(pt1);
  points2->InsertNextPoint(pt2);
  points2->InsertNextPoint(pt3);
  points2->InsertNextPoint(pt4);

  vtkSmartPointer<vtkPolyData> data2 = vtkSmartPointer<vtkPolyData>::New();
  data2 = vtk_viewer::cutMesh(data, points2, false);

  // Set input mesh
  tool_path_planner::SurfaceWalkRasterGenerator planner;
  planner.setInput(data2);

  // Set input tool data
  tool_path_planner::SurfaceWalkRasterGenerator::Config tool;
  tool.point_spacing = 0.6;
  tool.raster_spacing = 0.75;
  tool.tool_offset = 0.0;                // currently unused
  tool.intersection_plane_height = 0.2;  // 0.5 works best, not sure if this should be included in the tool
  tool.min_hole_size = 0.1;
  planner.setConfiguration(tool);

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 0.5;
  // Display mesh results
  color[0] = 0.9f;
  color[1] = 0.9f;
  color[2] = 0.9f;
  viz.addPolyDataDisplay(data2, color);

  // Display surface normals
  if (DISPLAY_NORMALS)
  {
    color[0] = 0.9f;
    color[1] = 0.1f;
    color[2] = 0.1f;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInput();
    viz.addPolyNormalsDisplay(normals_data, color, scale);
  }

  boost::optional<tool_path_planner::ToolPaths> paths = planner.generate();
  ASSERT_TRUE(paths);

  // Create sequence planner and set the data
  path_sequence_planner::SimplePathSequencePlanner sequence_planner;
  sequence_planner.setPaths(paths.get());
  sequence_planner.linkPaths();

  tool_path_planner::ToolPaths paths2 = sequence_planner.getPaths();
  std::vector<std::size_t> indices = sequence_planner.getIndices();
  tool_path_planner::ToolPathsData paths2_data = tool_path_planner::toToolPathsData(paths2);

  vtkSmartPointer<vtkPolyData> connecting_data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> connecting_points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkDoubleArray> normals = vtkSmartPointer<vtkDoubleArray>::New();
  normals->SetNumberOfComponents(3);
  for (std::size_t i = 0; i < paths2.size(); ++i)
  {
    for (std::size_t j = 0; j < paths2[i].size(); ++j)
    {
      if (DISPLAY_LINES)  // display line
      {
        color[0] = 0.2f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths2_data[i][j].line, color, scale);
      }

      if (DISPLAY_DERIVATIVES)  // display derivatives
      {
        color[0] = 0.9f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths2_data[i][j].derivatives, color, scale);
      }
    }

    if (i > 0)
    {
      const Eigen::Isometry3d& pt1 = paths2[i - 1].back().back();
      const Eigen::Isometry3d& pt2 = paths2[i].front().front();
      Eigen::VectorXd delta = (pt1.translation() - pt2.translation()).normalized();

      double pt[3];
      pt[0] = pt2.translation().x();
      pt[1] = pt2.translation().y();
      pt[2] = pt2.translation().z();
      connecting_points->InsertNextPoint(pt);

      double norm[3];
      norm[0] = delta.x();
      norm[1] = delta.y();
      norm[2] = delta.z();
      normals->InsertNextTuple(norm);
    }
  }
  connecting_data->SetPoints(connecting_points);
  connecting_data->GetPointData()->SetNormals(normals);

  color[0] = 0.2f;
  color[1] = 0.9f;
  color[2] = 0.9f;
  viz.addPolyNormalsDisplay(connecting_data, color, 1.0);

#ifdef NDEBUG
  // release build stuff goes here
  CONSOLE_BRIDGE_logError("noether/path_sequence_planner test: visualization is only available in debug mode");
#else
  // Debug-specific code goes here
  viz.renderDisplay();
#endif
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  // ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
