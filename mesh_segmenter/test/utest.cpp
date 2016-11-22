/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <gtest/gtest.h>
#include <mesh_segmenter/mesh_segmenter.h>


TEST(ViewerTest, TestCase1)
{
  std::string path = "/home/alex/path_planning_ws/src/Fillet_part_in.STL";
  //std::string path = "/home/alex/path_planning_ws/src/Top_Travel_Hardstop.STL";
  //std::string path = "/home/alex/path_planning_ws/src/Bracket.STL";
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane();
  //vtkSmartPointer<vtkPoints> points;
  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  // data = vtk_viewer::createMesh(points);

  data = vtk_viewer::readSTLFile(path);
  vtk_viewer::generateNormals(data);

  cout << data->GetCellData()->GetNormals()->GetNumberOfTuples() << "\n";

  mesh_segmenter::MeshSegmenter seg;
  seg.setInputMesh(data);
  seg.segmentMesh();

  std::vector<vtkSmartPointer<vtkPolyData> > meshes = seg.getMeshSegments();
  //copy_data->DeepCopy(data);
  //copy_data = vtk_viewer::upsampleMesh(data, 1.0);
  //vtk_viewer::visualizePlane(data);

  // Get curvature data
  vtkSmartPointer<vtkPolyData> data2 = vtkSmartPointer<vtkPolyData>::New();
  //data2 = vtk_viewer::estimateCurvature(data, 2);



  //cout << data2->GetPoints()->GetNumberOfPoints() << "\n";

  vtk_viewer::VTKViewer viz;

  std::vector<float> color(3);


  cout << "number of connected surfaces found: " << meshes.size() << "\n";
  // Display mesh results
  for(int i = 0; i < meshes.size(); ++i)
  {
    //vtkSmartPointer<vtkPolyData> copy_data = vtkSmartPointer<vtkPolyData>::New();
    color[0] = 0.1;
    color[1] = 0.5*double(i)/double(meshes.size());
    color[2] = 0.9*double(i)/double(meshes.size());

    //copy_data->DeepCopy(meshes[i]);
    //vtk_viewer::generateNormals(copy_data);
    //copy_data = vtk_viewer::upsampleMesh(copy_data, 0.5);
    //viz.addPolyDataDisplay(copy_data, color);
    viz.addPolyDataDisplay(meshes[i], color);
    viz.addCellNormalDisplay(meshes[i], color);
  }
  //vtkSmartPointer<vtkPoints> points2 = copy_data->GetPoints();
  //viz.addPointDataDisplay(points2, color);

  vtkSmartPointer<vtkPolyData> copy_data = vtkSmartPointer<vtkPolyData>::New();
  copy_data = seg.getInputMesh();
  //data2 = vtk_viewer::estimateCurvature(copy_data, 2);
  //viz.addPolyDataDisplay(data2, color);

  // Display mesh results
  color[0] = 0.2;
  color[1] = 0.2;
  color[2] = 0.9;
  //viz.addCellNormalDisplay(copy_data, color);
  color[0] = 0.9;
  color[1] = 0.2;
  color[2] = 0.9;
  //viz.addPolyDataDisplay(data, color);
  vtkSmartPointer<vtkPoints> points3 = data->GetPoints();
  viz.addPointDataDisplay(points3, color);


  viz.renderDisplay();

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
