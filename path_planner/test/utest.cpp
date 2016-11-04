
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

  // Display results
  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;

  //float color[3] {0.5, 0.5, 0.5};

  viz.addPolyDataDisplay(data, color);
  //std::vector<double> color2[3] (0.9, 0.5, 0.5);
  color[0] = 0.7;
  color[1] = 0.2;
  color[2] = 0.2;

  viz.addPolyDataDisplay(intersect_data, color);

  // Get points?
  color[0] = 0.2;
  color[1] = 0.2;
  color[2] = 0.9;
  vtkSmartPointer<vtkPolyData> smooth_data;
  vtkSmartPointer<vtkPoints> points, points2;
  vtkSmartPointer<vtkCellArray> lines;
  points = intersect_data->GetPoints();
  //lines = intersect_data->GetLines();
  cout << "number of intersect points : " << float(points->GetNumberOfPoints()) << "\n";

//  lines->InitTraversal();
//  cout << lines->GetNumberOfConnectivityEntries()  << "\n";


//  vtkSmartPointer<vtkIdList> pt = vtkSmartPointer<vtkIdList>::New();
//  vtkSmartPointer<vtkIdTypeArray> pt2 = vtkSmartPointer<vtkIdTypeArray>::New();
//  pt2 = lines->GetData();

  //vtkIdList* pt;
  //lines->GetCell(1,pt);
  //cout << pt->GetNumberOfIds()  << "\n";

//  int cell = lines->GetNextCell(pt);
//  int count = 0;
//  while(cell != 0)
//  {
//    //cout << cell << "\n";
//    if (pt->GetNumberOfIds() == 2)
//    {
//      cout << pt->GetId(0) << " " <<  pt->GetId(1) << "    ";

//      double* data2 = pt2->GetTuple(count);

//      cout << *data2 << "\n";
//      //cout << pt2[2] << "\n";
//    }
//    else
//    {
//      cout << pt->GetNumberOfIds()  << "\n";
//    }
//    cell = lines->GetNextCell(pt);
//    //cout << pt->GetNumberOfIds()  << "\n";
//    ++count;

//  }
//  cout << count;

  viz.addPointDataDisplay(points, color);

  smooth_data = planner.smoothData(points);

  color[0] = 0.2;
  color[1] = 0.9;
  color[2] = 0.9;

  viz.addPolyDataDisplay(smooth_data, color);

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
