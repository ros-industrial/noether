/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * file utest.cpp
 * All rights reserved.
 * copyright Copyright (c) 2019, Southwest Research Institute
 *
 * License
 * Software License Agreement (Apache License)
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
#include <ros/ros.h>
#include <noether_simulator/noether_simulator.h>
#include <vtk_viewer/vtk_utils.h>
#include <tool_path_planner/raster_tool_path_planner.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/PolygonMesh.h>

#include <vtkPointData.h>
#include <gtest/gtest.h>

// This tests runs the noether simulator
// it displays the mesh and path, then it
// displays a "heat" map that shows the
// result of painting the mesh using the
// given path.

//This test uses poor path to paint mesh
TEST(ViewerTest, TestCase1)
{

  // Create mesh to work with
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane(19,10,vtk_viewer::SINUSOIDAL_1D);

  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  data = vtk_viewer::createMesh(points, 0.5, 5);

  vtk_viewer::generateNormals(data);

  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  double pt1[3] = {3.0, 3.0, 0.0};
  double pt2[3] = {4.0, 2.0, 0.0};
  double pt3[3] = {5.0, 3.0, 0.0};
  double pt4[3] = {4.0, 4.0, 0.0};

  points2->InsertNextPoint(pt1);
  points2->InsertNextPoint(pt2);
  points2->InsertNextPoint(pt3);
  points2->InsertNextPoint(pt4);

  vtkSmartPointer<vtkPolyData> cut = vtkSmartPointer<vtkPolyData>::New();
  cut = vtk_viewer::cutMesh(data, points2, false);

  cout << "cutter points: " << cut->GetPoints()->GetNumberOfPoints() << "\n";
  cout << "cutter polys: " << cut->GetNumberOfPolys() << "\n";

  // Run tool path planner on mesh
  tool_path_planner::RasterToolPathPlanner planner;
  planner.setInputMesh(cut);

  tool_path_planner::ProcessTool tool;
  tool.pt_spacing = 0.5;
  tool.line_spacing = 0.75;
  tool.tool_offset = 0.0; // currently unused
  tool.intersecting_plane_height = 0.5; // 0.5 works best, not sure if this should be included in the tool
  tool.simulator_nearest_neighbors = 30; // not sure if this should be a part of the tool
  tool.min_hole_size = 0.1;
  tool.min_segment_size = 1;
  tool.raster_angle = 0;
  tool.raster_wrt_global_axes = 0;
  tool.simulator_tool_radius = 1;
  tool.simulator_tool_height = 2;
  planner.setTool(tool);
  planner.setDebugMode(false);

  planner.computePaths();

  // The NDEBUG variable is defined by the C/C++ standard when building
  // "Not Debug" (e.g. release).  NDEBUG is not defined during debug
  // builds - so the below code runs when building debug
  #ifndef NDEBUG
  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 1.0;

  // Display mesh results
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;
  viz.addPolyDataDisplay(cut, color);

  std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();

  for(int i = 0; i < paths.size(); ++i)
  {
  color[0] = 0.2;
  color[1] = 0.9;
  color[2] = 0.2;
  viz.addPolyNormalsDisplay(paths[i].line, color, scale);

  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.2;
  viz.addPolyNormalsDisplay(paths[i].derivatives, color, scale);
  }
  viz.renderDisplay();
  #endif
  // Run simulator on mesh and tool paths

  noether_simulator::NoetherSimulator simulator;
  simulator.setInputMesh(cut);
  simulator.setInputPaths(paths);
  simulator.setTool(tool);
  simulator.setDebugMode(true);

  simulator.runSimulation();

  vtkSmartPointer<vtkPolyData> processedPoints = vtkSmartPointer<vtkPolyData>::New();
  processedPoints = simulator.getSimulatedPoints();//get a copy of simulated points to operate on

  vtkIdType length = processedPoints->GetNumberOfPoints();
  double p[3];
  double intensity[3];
  int missed = 0;

  for(vtkIdType i = 0; i < length; i++)//iterate through points to check if covered
  {
    intensity[0] = processedPoints->GetPointData()->GetScalars()->GetComponent(i,0);
    intensity[1] = processedPoints->GetPointData()->GetScalars()->GetComponent(i,1);
    intensity[2] = processedPoints->GetPointData()->GetScalars()->GetComponent(i,2);

    if((intensity[0]+intensity[1]+intensity[2])/3.0 < 5) //averge of each pixel if less than threshold incrament counter
    {
      missed++;
    }

  }
  bool coverage;
  if(missed/float(processedPoints->GetNumberOfPoints())>0.1)//get ratio of missed spots, if below threshold set not covered
  {
    coverage = false;
  }
  else coverage = true;
  EXPECT_FALSE(coverage);//expect partial path to fail
}

//This test uses a good path to paint mesh
TEST(ViewerTest, TestCase2)
{

  // Create mesh to work with
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane(19,10,vtk_viewer::SINUSOIDAL_1D);
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  data = vtk_viewer::createMesh(points, 0.5, 5);

  vtk_viewer::generateNormals(data);

  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  double pt1[3] = {3.0, 3.0, 0.0};
  double pt2[3] = {4.0, 2.0, 0.0};
  double pt3[3] = {5.0, 3.0, 0.0};
  double pt4[3] = {4.0, 4.0, 0.0};

  points2->InsertNextPoint(pt1);
  points2->InsertNextPoint(pt2);
  points2->InsertNextPoint(pt3);
  points2->InsertNextPoint(pt4);

  vtkSmartPointer<vtkPolyData> cut = vtkSmartPointer<vtkPolyData>::New();
  cut = vtk_viewer::cutMesh(data, points2, false);

  cout << "cutter points: " << cut->GetPoints()->GetNumberOfPoints() << "\n";
  cout << "cutter polys: " << cut->GetNumberOfPolys() << "\n";

  // Run tool path planner on mesh
  tool_path_planner::RasterToolPathPlanner planner;
  planner.setInputMesh(cut);

  tool_path_planner::ProcessTool tool;
  tool.pt_spacing = 0.5;
  tool.line_spacing = 0.75;
  tool.tool_offset = 0.0; // currentlyc unused
  tool.intersecting_plane_height = 1.95; // 0.5 works best, not sure if this should be included in the tool
  tool.simulator_nearest_neighbors = 30; // not sure if this should be a part of the tool
  tool.min_hole_size = 0.1;
  tool.min_segment_size = 1;
  tool.raster_angle = 0;
  tool.raster_wrt_global_axes = 0;
  tool.simulator_tool_radius = 1;
  tool.simulator_tool_height = 2;
  planner.setTool(tool);
  planner.setDebugMode(false);

  planner.computePaths();

  // The NDEBUG variable is defined by the C/C++ standard when building
  // "Not Debug" (e.g. release).  NDEBUG is not defined during debug
  // builds - so the below code runs when building debug
  #ifndef NDEBUG
  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 1.0;

  // Display mesh results
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;
  viz.addPolyDataDisplay(cut, color);

  std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();

  for(int i = 0; i < paths.size(); ++i)
  {
  color[0] = 0.2;
  color[1] = 0.9;
  color[2] = 0.2;
  viz.addPolyNormalsDisplay(paths[i].line, color, scale);

  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.2;
  viz.addPolyNormalsDisplay(paths[i].derivatives, color, scale);
  }
  viz.renderDisplay();//display planned paths on mesh
  #endif
  // Run simulator on mesh and tool paths

  noether_simulator::NoetherSimulator simulator;
  simulator.setInputMesh(cut);
  simulator.setInputPaths(paths);
  simulator.setTool(tool);
  simulator.setDebugMode(true);
  simulator.runSimulation();

  vtkSmartPointer<vtkPolyData> processedPoints = vtkSmartPointer<vtkPolyData>::New();
  processedPoints = simulator.getSimulatedPoints();//get a copy of simulated points to operate on

  vtkIdType length = processedPoints->GetNumberOfPoints();
  double p[3];
  double intensity[3];
  int missed = 0;
  bool coverage;

  for(vtkIdType i = 0; i < length; i++)//iterate through points to check if covered
  {

    intensity[0] = processedPoints->GetPointData()->GetScalars()->GetComponent(i,0);
    intensity[1] = processedPoints->GetPointData()->GetScalars()->GetComponent(i,1);
    intensity[2] = processedPoints->GetPointData()->GetScalars()->GetComponent(i,2);

    if((intensity[0]+intensity[1]+intensity[2])/3.0 < 5) //averge of each pixel if less than threshold incrament counter
    {
      missed++;
    }
  }
  if(missed/float(processedPoints->GetNumberOfPoints())>0.1)//get ratio of missed spots, if below threshold set not covered
  {
    coverage = false;
  }
  else coverage = true;
  EXPECT_TRUE(coverage);//expect good mesh to pass
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
