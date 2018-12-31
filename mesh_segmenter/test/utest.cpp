/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

//#include <vtk_viewer/vtk_utils.h>
//#include <vtk_viewer/vtk_viewer.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkPlanes.h>

#include <gtest/gtest.h>
#include <mesh_segmenter/mesh_segmenter.h>

// This whole test has been commented 12/31/2018 1) because it doesn't work with Travis and 2) Because it depends on vtk_viewer which is unnecessary for this library.
// It should/will be replaced by something with Gtest pass/fail criteria.

// This test displays the output of the mesh segmenter as applied to a cube.  Result should be
// a cube with each face colored a different color.

TEST(ViewerTest, TestCase1)
{
//  // generate object
//  vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
//  cubeSource->SetBounds(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0);
//  cubeSource->Update();

//  // generate normals
//  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
//  data = cubeSource->GetOutput();
//  vtk_viewer::generateNormals(data);

//  // segment
//  mesh_segmenter::MeshSegmenter seg;
//  seg.setInputMesh(data);
//  seg.segmentMesh();
//  std::vector<vtkSmartPointer<vtkPolyData> > meshes = seg.getMeshSegments();

//  // display
//  vtk_viewer::VTKViewer viz;

//  // Display mesh results
//  int colors[] = {
//    0xcc0000,
//    0xcc6500,
//    0xcccc00,
//    0x65cc00,
//    0x00cc00,
//    0x00cc65,
//    0x00cccc,
//    0x0065cc,
//    0x0000cc,
//    0x6500cc,
//    0xcc00cc,
//    0xcc0065,
//    };

//  size_t size;
//  size=sizeof(colors)/sizeof(colors[0]);

//  for(int i = 0; i < meshes.size(); ++i)
//  {
//    std::vector<float> color(3);
//    color[2] = float(colors[i % size] & 0xff)/255.0;
//    color[1] = float((colors[i % size] & 0xff00) >> 8)/255.0;
//    color[0] = float((colors[i % size] & 0xff0000) >> 16)/255.0;

//    viz.addPolyDataDisplay(meshes[i], color);
//  }

//  viz.renderDisplay();

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
