/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <vtkPointData.h>
#include <gtest/gtest.h>


TEST(ViewerTest, TestCase1)
{
  std::string path = "/home/alex/path_planning_ws/src/Fillet_part_in.STL";
  //std::string path = "/home/alex/path_planning_ws/src/Top_Travel_Hardstop.STL";
  //std::string path = "/home/alex/path_planning_ws/src/Bracket.STL";
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane();
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  points2->DeepCopy(points);
  //vtkSmartPointer<vtkPoints> points;
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
   data = vtk_viewer::createMesh(points);

  //data = vtk_viewer::readSTLFile(path);
  vtk_viewer::generateNormals(data);

  vtkSmartPointer<vtkPolyData> copy_data = vtkSmartPointer<vtkPolyData>::New();
  //copy_data->DeepCopy(data);
  //copy_data = vtk_viewer::upsampleMesh(data, 1.0);
  //vtk_viewer::visualizePlane(data);

  // Get curvature data
  vtkSmartPointer<vtkPolyData> data2 = vtkSmartPointer<vtkPolyData>::New();
  //data2 = vtk_viewer::estimateCurvature(data, 2);



  //cout << data2->GetPoints()->GetNumberOfPoints() << "\n";

  vtk_viewer::VTKViewer viz;

  std::vector<float> color(3);


  // Display mesh results
  color[0] = 0.2;
  color[1] = 0.9;
  color[2] = 0.9;
  //viz.addPolyDataDisplay(copy_data, color);
  //vtkSmartPointer<vtkPoints> points2 = copy_data->GetPoints();
  viz.addPointDataDisplay(points2, color);

  //data2 = vtk_viewer::estimateCurvature(copy_data, 2);
  viz.addPolyDataDisplay(data, color);

  // Display mesh results
  color[0] = 0.2;
  color[1] = 0.2;
  color[2] = 0.9;
  //viz.addCellNormalDisplay(data, color);
  //viz.addPolyDataDisplay(data, color);
  //vtkSmartPointer<vtkPoints> points3 = data->GetPoints();
  //viz.addPointDataDisplay(points3, color);


  viz.renderDisplay();

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
