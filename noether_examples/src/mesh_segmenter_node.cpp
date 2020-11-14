/*
 * Apache 2.0
 *
 */

#include <mesh_segmenter/mesh_segmenter.h>
#include <vtkPointData.h>
#include <ros/ros.h>
#include <ros/file_log.h>

#include <vtkPolyDataNormals.h>
#include <vtkCleanPolyData.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <vtkSmoothPolyDataFilter.h>

#include <noether_examples/noether_examples.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtkSTLWriter.h>
#include <ros/package.h>

namespace noether
{
void Noether::addMeshDisplay(std::vector<vtkSmartPointer<vtkPolyData> >& meshes)
{
  // mesh colors should be darker than path colors
  int colors[] = {
    0xcc0000, 0xcc6500, 0xcccc00, 0x65cc00, 0x00cc00, 0x00cc65,
    0x00cccc, 0x0065cc, 0x0000cc, 0x6500cc, 0xcc00cc, 0xcc0065,
  };

  size_t size;
  size = sizeof(colors) / sizeof(colors[0]);

  for (int i = 0; i < meshes.size(); ++i)
  {
    std::vector<float> color(3);
    color[2] = float(colors[i % size] & 0xff) / 255.0;
    color[1] = float((colors[i % size] & 0xff00) >> 8) / 255.0;
    color[0] = float((colors[i % size] & 0xff0000) >> 16) / 255.0;

    viewer_.addPolyDataDisplay(meshes[i], color);
  }
}
}  // namespace noether

static std::string toLower(const std::string& in)
{
  std::string copy = in;
  std::transform(copy.begin(), copy.end(), copy.begin(), ::tolower);
  return copy;
}

static bool readFile(std::string& file, std::vector<vtkSmartPointer<vtkPolyData> >& mesh)
{
  if (!file.empty())
  {
    // read data file
    vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
    std::vector<char> buffer(file.size() + 1, '\0');
    char* str = buffer.data();
    strcpy(str, file.c_str());

    char* pch;
    pch = strtok(str, " ,.-");
    while (pch != NULL)
    {
      std::string extension(pch);
      if (extension == "pcd" || extension == "stl" || extension == "STL" || extension == "ply")
      {
        break;
      }
      pch = strtok(NULL, " ,.-");
    }

    std::string extension = std::string(pch);
    if (extension == "pcd")
    {
      vtk_viewer::loadPCDFile(file, data);
    }
    else if (extension == "STL" || extension == "stl")
    {
      data = vtk_viewer::readSTLFile(file);
    }
    else if (toLower(extension) == "ply")  // PCL polygon mesh
    {
      pcl::PolygonMesh pcl_mesh;
      vtk_viewer::loadPolygonMeshFromPLY(file, pcl_mesh);
      vtk_viewer::pclEncodeMeshAndNormals(pcl_mesh, data);
    }
    else
    {
      ROS_ERROR("Unrecognized extension: '%s'. Program supports 'pcd', 'stl', 'STL', 'ply'", extension.c_str());
      return false;
    }

    vtk_viewer::generateNormals(data);
    mesh.push_back(data);
    return true;
  }
  else
  {
    ROS_WARN_STREAM("'filename' parameter must be set to the path of a pcd or stl file");
    return false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_segmenter_node");
  ros::NodeHandle pnh("~");

  // Step 1: Load parameters
  std::string file;
  double curvature_threshold;
  int min_cluster_size, max_cluster_size;
  bool show_individually, save_outputs;
  pnh.param<std::string>("filename", file, "");
  pnh.param<int>("min_cluster_size", min_cluster_size, 500);
  pnh.param<int>("max_cluster_size", max_cluster_size, 1000000);
  pnh.param<double>("curvature_threshold", curvature_threshold, 0.3);
  pnh.param<bool>("show_individually", show_individually, false);
  pnh.param<bool>("save_outputs", save_outputs, false);

  std::vector<vtkSmartPointer<vtkPolyData> > meshes;
  readFile(file, meshes);

  // Step 2: Filter the mesh
  // Create some pointers - not used since passing with VTK pipeline but useful for debugging
  ROS_INFO("Beginning Filtering.");
  vtkSmartPointer<vtkPolyData> mesh_in = meshes[0];
  vtkSmartPointer<vtkPolyData> mesh_cleaned;
  vtkSmartPointer<vtkPolyData> mesh_filtered1;
  vtkSmartPointer<vtkPolyData> mesh_filtered2;

  // Apply Windowed Sinc function interpolation Smoothing
  vtkSmartPointer<vtkWindowedSincPolyDataFilter> smooth_filter1 = vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
  smooth_filter1->SetInputData(mesh_in);
  smooth_filter1->SetNumberOfIterations(20);
  smooth_filter1->SetPassBand(0.1);
  smooth_filter1->FeatureEdgeSmoothingOff();  // Smooth along sharp interior edges
  smooth_filter1->SetFeatureAngle(45);        // Angle to identify sharp edges (degrees)
  smooth_filter1->SetEdgeAngle(15);           // Not sure what this controls (degrees)
  smooth_filter1->BoundarySmoothingOff();
  smooth_filter1->NonManifoldSmoothingOff();
  smooth_filter1->NormalizeCoordinatesOn();  // "Improves numerical stability"
  smooth_filter1->Update();
  mesh_filtered1 = smooth_filter1->GetOutput();

  // Apply Laplacian Smoothing
  // This moves the coordinates of each point toward the average of its adjoining points
  vtkSmartPointer<vtkSmoothPolyDataFilter> smooth_filter2 = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
  smooth_filter2->SetInputConnection(smooth_filter1->GetOutputPort());
  smooth_filter2->SetNumberOfIterations(10);
  smooth_filter2->SetRelaxationFactor(0.1);
  smooth_filter2->FeatureEdgeSmoothingOff();
  smooth_filter2->BoundarySmoothingOff();
  smooth_filter2->Update();
  mesh_filtered2 = smooth_filter2->GetOutput();

  // Step 3: Segment the mesh
  mesh_segmenter::MeshSegmenter segmenter;
  //  segmenter.setInputMesh(mesh_in);                           // Use to ignore filters
  segmenter.setInputMesh(mesh_filtered2);
  segmenter.setMinClusterSize(min_cluster_size);
  segmenter.setMaxClusterSize(max_cluster_size);
  segmenter.setCurvatureThreshold(curvature_threshold);

  ROS_INFO("Beginning Segmentation.");
  ros::Time tStart = ros::Time::now();
  segmenter.segmentMesh();
  ROS_INFO("Segmentation time: %.3f", (ros::Time::now() - tStart).toSec());

  // Get Mesh segment
  std::vector<vtkSmartPointer<vtkPolyData> > segmented_meshes = segmenter.getMeshSegments();
  std::vector<vtkSmartPointer<vtkPolyData> > panels(segmented_meshes.begin(), segmented_meshes.end() - 1);

  ROS_INFO_STREAM("Displaying " << segmented_meshes.size() << " meshes \n");
  ROS_INFO("Close VTK window to continue");
  if (show_individually)
  {
    for (int ind = 0; ind < segmented_meshes.size(); ind++)
    {
      if (true)
      {
        ROS_INFO_STREAM("Mesh: " << ind << "\n");
        std::vector<vtkSmartPointer<vtkPolyData> > tmp(1);
        tmp.push_back(segmented_meshes[ind]);
        noether::Noether viz;
        viz.addMeshDisplay(tmp);
        viz.visualizeDisplay();
      }
    }
  }
  else
  {
    noether::Noether viz;
    ROS_INFO("Displaying Segments");
    viz.addMeshDisplay(segmented_meshes);
    viz.visualizeDisplay();
  }

  if (save_outputs)
  {
    for (int ind = 0; ind < segmented_meshes.size(); ind++)
    {
      ROS_INFO_STREAM("Saving: " << ind << "\n");

      std::ostringstream ss;
      ss << ind;
      std::string filename = ros::package::getPath("noether_examples") + "/meshes/outputs/output_" + ss.str() + ".stl";

      vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
      stlWriter->SetFileName(filename.c_str());
      stlWriter->SetInputData(segmented_meshes[ind]);
      stlWriter->Write();
    }
  }

  return 0;
}
