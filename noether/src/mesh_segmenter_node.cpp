/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "noether/noether.h"
#include <mesh_segmenter/mesh_segmenter.h>
#include <vtkPointData.h>
#include <ros/ros.h>
#include <ros/file_log.h>

static std::string toLower(const std::string& in)
{
  std::string copy = in;
  std::transform(copy.begin(), copy.end(), copy.begin(), ::tolower);
  return copy;
}

static bool readFile(std::string file, std::vector<vtkSmartPointer<vtkPolyData> > mesh)
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
//      if (argc == 3)
//      {
//        std::string background = argv[2];
//        vtk_viewer::loadPCDFile(file, data, background, true);
//      }
//      else
//      {
        vtk_viewer::loadPCDFile(file, data);
//      }
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
  ros::init(argc, argv, "noether_node");
  ros::NodeHandle pnh("~");

  // Step 1: Load the 'filename' parameter
  std::string file;
  double curvature_threshold;
  int min_cluster_size, max_cluster_size;
  pnh.param<std::string>("filename", file, "");
  pnh.param<int>("min_cluster_size",min_cluster_size, 500);
  pnh.param<int>("max_cluster_size",max_cluster_size, 1000000);
  pnh.param<double>("curvature_threshold",curvature_threshold, 0.8);

  std::vector<vtkSmartPointer<vtkPolyData> > mesh;
  readFile(file, mesh);

  // ------- Segment mesh ---------
  // Instantiate a segmenter
  mesh_segmenter::MeshSegmenter segmenter;

  // Loop over all of the meshes in the vector (should only be one if fully connected)

  // Set up the segmenter
  segmenter.setInputMesh(mesh[0]);
  segmenter.setMinClusterSize(min_cluster_size);
  segmenter.setMaxClusterSize(max_cluster_size);
  segmenter.setCurvatureThreshold(curvature_threshold);

  segmenter.segmentMesh();

  // Get Mesh segment
  std::vector<vtkSmartPointer<vtkPolyData> > segmented_meshes = segmenter.getMeshSegments();

  std::cout << segmented_meshes.size() << '\n';

  noether::Noether viz;
  viz.addMeshDisplay(segmented_meshes);
  viz.visualizeDisplay();

  //    std::string log_directory = ros::file_log::getLogDirectory();

  //    // plan paths for segmented meshes
  //    tool_path_planner::ProcessTool tool = loadTool(pnh);
  //    tool_path_planner::RasterToolPathPlanner planner(tool.use_ransac_normal_estimation);

  //    bool debug_on;
  //    pnh.param<bool>("debug_on", debug_on, false);
  //    double vect[3], center[3];
  //    pnh.param<double>("cut_norm_x", vect[0], 0.0);
  //    pnh.param<double>("cut_norm_y", vect[1], 0.0);
  //    pnh.param<double>("cut_norm_z", vect[2], 0.0);
  //    pnh.param<double>("centroid_x", center[0], 0.0);
  //    pnh.param<double>("centroid_y", center[1], 0.0);
  //    pnh.param<double>("centroid_z", center[2], 0.0);

  //    planner.setTool(tool);
  //    planner.setCutDirection(vect);
  //    planner.setCutCentroid(center);
  //    planner.setDebugMode(debug_on);
  //    planner.setLogDir(log_directory);
  //    std::vector<std::vector<tool_path_planner::ProcessPath> > paths;
  //    planner.planPaths(meshes, paths);

  //    // visualize results
  //    double scale = tool.pt_spacing * 1.5;
  //    noether::Noether viz;
  //    viz.setLogDir(log_directory);
  //    ROS_INFO_STREAM("log directory " << viz.getLogDir());

  //    viz.addMeshDisplay(meshes);
  //    viz.addPathDisplay(paths, scale, true, false, false);
  //    viz.visualizeDisplay();

  return 0;
}
