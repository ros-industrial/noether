/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */



#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <gtest/gtest.h>
#include <vtkIdTypeArray.h>

#include "../include/tool_path_planner/raster_tool_path_planner.h"

#define DISPLAY_LINES  1
#define DISPLAY_NORMALS  0
#define DISPLAY_DERIVATIVES  1
#define DISPLAY_CUTTING_MESHES  0
#define POINT_SPACING 0.5


TEST(IntersectTest, RasterRotationTest)
{
  double raster_directions[] = {0., 0.78, 1.57, 2.35};

  for (auto direction : raster_directions)
  {
    // Get mesh
    vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane(30, 10, vtk_viewer::FLAT);
    vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(points, 0.5, 5);
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
    tool_path_planner::RasterToolPathPlanner planner;
    planner.setInputMesh(data2);

    // Set input tool data
    tool_path_planner::ProcessTool tool;
    tool.pt_spacing = POINT_SPACING;
    tool.line_spacing = 0.75;
    tool.tool_offset = 0.0;                // currently unused
    tool.intersecting_plane_height = 0.2;  // 0.5 works best, not sure if this should be included in the tool
    tool.min_hole_size = 0.1;
    planner.setTool(tool);
    planner.setDebugMode(false);
    planner.setRasterAngle(direction);
    planner.setRasterWRTPrincipalAxis(true);

    vtk_viewer::VTKViewer viz;
    std::vector<float> color(3);

    double scale = 1.0;

    // Display mesh results
    color[0] = 0.9;
    color[1] = 0.9;
    color[2] = 0.9;
    viz.addPolyDataDisplay(data2, color);

    // Display surface normals
    if (DISPLAY_NORMALS)
    {
      color[0] = 0.9;
      color[1] = 0.1;
      color[2] = 0.1;
      vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
      normals_data = planner.getInputMesh();
      viz.addPolyNormalsDisplay(normals_data, color, scale);
    }

    // Plan paths for given mesh
    planner.computePaths();
    std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();

    for (int i = 0; i < paths.size(); ++i)
    {
      if (DISPLAY_LINES)  // display line
      {
        color[0] = 0.2;
        color[1] = 0.9;
        color[2] = 0.2;
        viz.addPolyNormalsDisplay(paths[i].line, color, scale);
      }

      if (DISPLAY_DERIVATIVES)  // display derivatives
      {
        color[0] = 0.9;
        color[1] = 0.9;
        color[2] = 0.2;
        viz.addPolyNormalsDisplay(paths[i].derivatives, color, scale);
      }

      if (DISPLAY_CUTTING_MESHES)  // Display cutting mesh
      {
        color[0] = 0.9;
        color[1] = 0.9;
        color[2] = 0.9;
        viz.addPolyDataDisplay(paths[i].intersection_plane, color);
      }
    }

#ifdef NDEBUG
    // release build stuff goes here
    LOG4CXX_ERROR(tool_path_planner::RasterToolPathPlanner::RASTER_PATH_PLANNER_LOGGER,
                  "noether/tool_path_planner test: visualization is only available in debug mode");
#else
    // Debug-specific code goes here
    viz.renderDisplay();
#endif
  }
}

// This test shows the results of the tool path planner on a square grid that has a sinusoidal
// variation in the z axis and a cutout in the middle.  It will generate a series of evenly spaced lines (aprox. equal to line_spacing)
// with evenly spaced points on each line (aprox. equal to pt_spacing).  Yellow arrows show the direction of
// travel in a give line (all lines should point in the same direction) and green arrows show the process normal
// direction (should be aprox. normal to the surface in that region)

TEST(IntersectTest, TestCase1)
{
  // Get mesh
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane();
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(points, 0.5, 5);
  vtk_viewer::generateNormals(data);


  // create cutout in the middle of the mesh
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  double pt1[3] = {2.0, 3.0, 0.0};
  double pt2[3] = {4.0, 2.0, 0.0};
  double pt3[3] = {5.0, 3.0, 0.0};
  double pt4[3] = {4.0, 5.0, 0.0};
  points2->InsertNextPoint(pt1);
  points2->InsertNextPoint(pt2);
  points2->InsertNextPoint(pt3);
  points2->InsertNextPoint(pt4);

  vtkSmartPointer<vtkPolyData> data2 = vtkSmartPointer<vtkPolyData>::New();
  data2 = vtk_viewer::cutMesh(data, points2, false);


  // Set input mesh
  tool_path_planner::RasterToolPathPlanner planner;
  planner.setInputMesh(data2);

  // Set input tool data
  tool_path_planner::ProcessTool tool;
  tool.pt_spacing = POINT_SPACING;
  tool.line_spacing = 0.75;
  tool.tool_offset = 0.0; // currently unused
  tool.intersecting_plane_height = 0.2; // 0.5 works best, not sure if this should be included in the tool
  tool.min_hole_size = 0.1;
  planner.setTool(tool);
  planner.setDebugMode(false);

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 1.0;

  // Display mesh results
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;
  viz.addPolyDataDisplay(data2, color);


  // Display surface normals
  if(DISPLAY_NORMALS)
  {
    color[0] = 0.9;
    color[1] = 0.1;
    color[2] = 0.1;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInputMesh();
    viz.addPolyNormalsDisplay(normals_data, color, scale);
  }


  // Plan paths for given mesh
  planner.computePaths();
  std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();

  for(int i = 0; i < paths.size(); ++i)
  {
    if(DISPLAY_LINES) // display line
    {
      color[0] = 0.2;
      color[1] = 0.9;
      color[2] = 0.2;
      viz.addPolyNormalsDisplay(paths[i].line, color, scale);
    }

    if(DISPLAY_DERIVATIVES) // display derivatives
    {
    color[0] = 0.9;
    color[1] = 0.9;
    color[2] = 0.2;
    viz.addPolyNormalsDisplay(paths[i].derivatives, color, scale);
    }

    if(DISPLAY_CUTTING_MESHES) // Display cutting mesh
    {
      color[0] = 0.9;
      color[1] = 0.9;
      color[2] = 0.9;
      viz.addPolyDataDisplay(paths[i].intersection_plane, color);
    }
  }

  #ifdef NDEBUG
  // release build stuff goes here
  LOG4CXX_ERROR(tool_path_planner::RasterToolPathPlanner::RASTER_PATH_PLANNER_LOGGER, "noether/tool_path_planner test: visualization is only available in debug mode");
  #else
  // Debug-specific code goes here
  viz.renderDisplay();
  #endif
}

TEST(IntersectTest, TestCaseRansac)
{
  // Get mesh
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane();
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(points, 0.5, 5);
  vtk_viewer::generateNormals(data);


  // create cutout in the middle of the mesh
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  double pt1[3] = {2.0, 3.0, 0.0};
  double pt2[3] = {4.0, 2.0, 0.0};
  double pt3[3] = {5.0, 3.0, 0.0};
  double pt4[3] = {4.0, 5.0, 0.0};
  points2->InsertNextPoint(pt1);
  points2->InsertNextPoint(pt2);
  points2->InsertNextPoint(pt3);
  points2->InsertNextPoint(pt4);

  vtkSmartPointer<vtkPolyData> data2 = vtkSmartPointer<vtkPolyData>::New();
  data2 = vtk_viewer::cutMesh(data, points2, false);


  // Set input mesh
  tool_path_planner::RasterToolPathPlanner planner;
  planner.setInputMesh(data2);

  // Set input tool data
  tool_path_planner::ProcessTool tool;
  tool.pt_spacing = POINT_SPACING;
  tool.line_spacing = 0.75;
  tool.tool_offset = 0.0; // currently unused
  tool.intersecting_plane_height = 0.2; // 0.5 works best, not sure if this should be included in the tool
  tool.min_hole_size = 0.1;
  planner.setTool(tool);
  planner.setDebugMode(false);

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 1.0;

  // Display mesh results
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;
  viz.addPolyDataDisplay(data2, color);


  // Display surface normals
  if(DISPLAY_NORMALS)
  {
    color[0] = 0.9;
    color[1] = 0.1;
    color[2] = 0.1;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInputMesh();
    viz.addPolyNormalsDisplay(normals_data, color, scale);
  }


  // Plan paths for given mesh
  planner.computePaths();
  std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();

  for(int i = 0; i < paths.size(); ++i)
  {
    if(DISPLAY_LINES) // display line
    {
      color[0] = 0.2;
      color[1] = 0.9;
      color[2] = 0.2;
      viz.addPolyNormalsDisplay(paths[i].line, color, scale);
    }

    if(DISPLAY_DERIVATIVES) // display derivatives
    {
    color[0] = 0.9;
    color[1] = 0.9;
    color[2] = 0.2;
    viz.addPolyNormalsDisplay(paths[i].derivatives, color, scale);
    }

    if(DISPLAY_CUTTING_MESHES) // Display cutting mesh
    {
      color[0] = 0.9;
      color[1] = 0.9;
      color[2] = 0.9;
      viz.addPolyDataDisplay(paths[i].intersection_plane, color);
    }
  }

  #ifdef NDEBUG
  // release build stuff goes here
  LOG4CXX_ERROR(tool_path_planner::RasterToolPathPlanner::RASTER_PATH_PLANNER_LOGGER, "noether/tool_path_planner test: visualization is only available in debug mode");
  #else
  // Debug-specific code goes here
  viz.renderDisplay();
  #endif
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
