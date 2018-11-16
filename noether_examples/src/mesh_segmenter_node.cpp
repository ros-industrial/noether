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

#include <vtk_viewer/vtk_utils.h>
#include <noether/noether.h>

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

  // Step 1: Load the 'filename' parameter
  std::string file;
  double curvature_threshold;
  int min_cluster_size, max_cluster_size;
  pnh.param<std::string>("filename", file, "");
  pnh.param<int>("min_cluster_size", min_cluster_size, 500);
  pnh.param<int>("max_cluster_size", max_cluster_size, 1000000);
  pnh.param<double>("curvature_threshold", curvature_threshold, 0.3);

  std::vector<vtkSmartPointer<vtkPolyData> > meshes;
  readFile(file, meshes);

  // Step 2: Filter the mesh
  // Create some pointers - not used since passing with "ports" but useful for debugging
  ROS_INFO("Beginning Filtering.");
  vtkSmartPointer<vtkPolyData> mesh_in = meshes[0];
  vtkSmartPointer<vtkPolyData> mesh_cleaned;
  vtkSmartPointer<vtkPolyData> mesh_filtered1;
  vtkSmartPointer<vtkPolyData> mesh_filtered2;

  // Remove duplicate points
  vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
  cleanPolyData->SetInputData(mesh_in);
  cleanPolyData->Update();
  mesh_cleaned = cleanPolyData->GetOutput();

  // Apply Windowed Sinc function interpolation Smoothing
  vtkSmartPointer<vtkWindowedSincPolyDataFilter> smooth_filter1 = vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
  smooth_filter1->SetInputConnection(cleanPolyData->GetOutputPort());
  smooth_filter1->SetInputData(mesh_cleaned);
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
  //  smooth_filter2->SetInputData(mesh_cleaned);
  smooth_filter2->SetNumberOfIterations(10);
  smooth_filter2->SetRelaxationFactor(0.1);
  //  smooth_filter2->SetEdgeAngle(somenumber);
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
  std::vector<vtkSmartPointer<vtkPolyData> > edges(1);
  edges.push_back(segmented_meshes.back());

  ROS_INFO("Displaying Edges");
  noether::Noether viz;
  viz.addMeshDisplay(edges);
  viz.visualizeDisplay();
  ROS_INFO("Displaying Segments");
  viz.addMeshDisplay(panels);
  viz.visualizeDisplay();
  ROS_INFO("Displaying Both");
  viz.addMeshDisplay(segmented_meshes);
  viz.visualizeDisplay();

  return 0;
}
