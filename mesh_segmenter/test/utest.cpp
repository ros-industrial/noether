#include <gtest/gtest.h>
#include <mesh_segmenter/mesh_segmenter.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>

#include <vtkPolyDataNormals.h>
#include <vtkCellData.h>

/**
 * @brief Creates a sinusoidal mesh and runs segmentation on it. The results are then checked
 *
 * Successful test will have the expected number of results and will have the same number of cells in the output as are
 * in the input
 */
TEST(SegmentationTest, TestCase1)
{
  int min_cluster_size = 500;
  int max_cluster_size = 1000;
  double curvature_threshold = 0.2;

  // Get mesh - This leaves some holes in it for some reason.
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane(20, 20, vtk_viewer::SINUSOIDAL_1D);
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(points, 0.25, 7);

  // Generate normals
  vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
  normalGenerator->SetInputData(data);
  normalGenerator->ComputePointNormalsOn();
  normalGenerator->ComputeCellNormalsOn();
  normalGenerator->NonManifoldTraversalOn();
  normalGenerator->Update();

  // Set up segmentation
  mesh_segmenter::MeshSegmenter seg;
  seg.setInputMesh(normalGenerator->GetOutput());
  seg.setMinClusterSize(min_cluster_size);
  seg.setMaxClusterSize(max_cluster_size);
  seg.setCurvatureThreshold(curvature_threshold);
  seg.segmentMesh();
  std::vector<vtkSmartPointer<vtkPolyData> > meshes = seg.getMeshSegments();

// The NDEBUG variable is defined by the C/C++ standard when building
// "Not Debug" (e.g. release).  NDEBUG is not defined during debug
// builds - so the below code runs when building debug
#ifndef NDEBUG
  // Display meshes for debugging
  vtk_viewer::VTKViewer viz;

  // Display mesh results
  int colors[] = {
    0xcc0000, 0xcc6500, 0xcccc00, 0x65cc00, 0x00cc00, 0x00cc65,
    0x00cccc, 0x0065cc, 0x0000cc, 0x6500cc, 0xcc00cc, 0xcc0065,
  };
  size_t size;
  size = sizeof(colors) / sizeof(colors[0]);

  for (std::size_t i = 0; i < meshes.size(); ++i)
  {
    std::vector<float> color(3);
    color[2] = float(colors[i % size] & 0xff) / 255.0;
    color[1] = float((colors[i % size] & 0xff00) >> 8) / 255.0;
    color[0] = float((colors[i % size] & 0xff0000) >> 16) / 255.0;

    viz.addPolyDataDisplay(meshes[i], color);
    // Debug-specific code goes here
    viz.renderDisplay();
  }
#endif

  // Check results
  EXPECT_EQ(meshes.size(), 7);
  long long cells_out = 0;
  long long cells_in = normalGenerator->GetOutput()->GetCellData()->GetNumberOfTuples();
  for (std::size_t ind = 0; ind < meshes.size(); ind++)
  {
    cells_out += meshes.at(ind)->GetNumberOfCells();
  }
  // There should be the same number of cells coming in as there are going out
  EXPECT_EQ(cells_in, cells_out);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
