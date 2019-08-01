/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
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
#include <vtkPointData.h>
#include <gtest/gtest.h>

// This test shows the results of meshing on a square grid that has a sinusoidal
// variability in the z axis.  Red arrows show the surface normal for each triangle
// in the mesh, and cyan boxes show the points used to seed the mesh creation algorithm

TEST(ViewerTest, TestCase1)
{
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane();

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

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  // Display mesh results
  color[0] = 0.2;
  color[1] = 0.9;
  color[2] = 0.9;

  viz.addPointDataDisplay(points, color);

  // Display mesh results
  color[0] = 0.2;
  color[1] = 0.2;
  color[2] = 0.9;
  //viz.addPolyDataDisplay(data, color);

  color[0] = 0.1;
  color[1] = 0.9;
  color[2] = 0.1;
  //viz.addCellNormalDisplay(data, color, 1.0);

  viz.addPolyDataDisplay(cut,color);
  viz.addPointDataDisplay(cut->GetPoints(), color);

  #ifdef NDEBUG
  // release build stuff goes here
  LOG4CXX_ERROR(vtk_viewer::VTK_LOGGER,"noether/vtk_viewer: visualization is only available in debug mode");
  #else
  // Debug-specific code goes here
  viz.renderDisplay();
  #endif
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
