/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "noether/noether.h"
#include <vtkPointData.h>
#include <ros/ros.h>
#include <ros/file_log.h>

namespace noether {

void Noether::addMeshDisplay(std::vector<vtkSmartPointer<vtkPolyData> >& meshes)
{
  // mesh colors should be darker than path colors
  int colors[] = {
    0xcc0000,
    0xcc6500,
    0xcccc00,
    0x65cc00,
    0x00cc00,
    0x00cc65,
    0x00cccc,
    0x0065cc,
    0x0000cc,
    0x6500cc,
    0xcc00cc,
    0xcc0065,
    };

  size_t size;
  size=sizeof(colors)/sizeof(colors[0]);

  for(int i = 0; i < meshes.size(); ++i)
  {
    std::vector<float> color(3);
    color[2] = float(colors[i % size] & 0xff)/255.0;
    color[1] = float((colors[i % size] & 0xff00) >> 8)/255.0;
    color[0] = float((colors[i % size] & 0xff0000) >> 16)/255.0;

    viewer_.addPolyDataDisplay(meshes[i], color);
  }
}

void Noether::addPathDisplay(std::vector<std::vector< tool_path_planner::ProcessPath > >& paths,
                    double scale, bool show_path, bool show_cutting_meshes, bool show_derivatives)
{
  for(int i = 0; i < paths.size(); ++i)
  {
    addPathDisplay(paths[i], scale, show_path, show_cutting_meshes, show_derivatives);
  }
}

void Noether::addPathDisplay(std::vector< tool_path_planner::ProcessPath >& paths, double scale, bool show_path,
                    bool show_cutting_meshes, bool show_derivatives)
{
  int colors[] = {
    0xff0000,
    0xff8000,
    0xffff00,
    0x80ff00,
    0x00ff00,
    0x00ff80,
    0x00ffff,
    0x0080ff,
    0x0000ff,
    0x8000ff,
    0xff00ff,
    0xff0080,
    };

  size_t size;
  size=sizeof(colors)/sizeof(colors[0]);
  std::vector<float> color(3);

  for(int i = 0; i < paths.size(); ++i)
  {
    if(show_path)
    {
      color[2] = float(colors[i % size] & 0xff)/255.0;
      color[1] = float((colors[i % size] & 0xff00) >> 8)/255.0;
      color[0] = float((colors[i % size] & 0xff0000) >> 16)/255.0;
      viewer_.addPolyNormalsDisplay(paths[i].line, color, scale);
    }
    if(show_cutting_meshes)
    {
      color[0] = 0.9;
      color[1] = 0.9;
      color[2] = 0.9;
      viewer_.addPolyDataDisplay(paths[i].intersection_plane, color);
    }
    if(show_derivatives)
    {
      color[0] = 0.9;
      color[1] = 0.9;
      color[2] = 0.2;
      viewer_.addPolyNormalsDisplay(paths[i].derivatives, color, scale);
    }
  }
}

}

tool_path_planner::ProcessTool loadTool(ros::NodeHandle& nh)
{
  tool_path_planner::ProcessTool tool;

  nh.param<double>("pt_spacing", tool.pt_spacing, 0.005);
  nh.param<double>("line_spacing", tool.line_spacing, 0.05);
  nh.param<double>("tool_offset", tool.tool_offset, 0);
  nh.param<double>("intersecting_plane_height", tool.intersecting_plane_height, 0.05);
  nh.param<double>("min_hole_size", tool.min_hole_size, 0.01);
  nh.param<double>("min_segment_size", tool.min_segment_size, 0.01);

  return tool;
}

static std::string toLower(const std::string& in)
{
  std::string copy = in;
  std::transform(copy.begin(), copy.end(), copy.begin(), ::tolower);
  return copy;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "noether_node");
  ros::NodeHandle pnh ("~");

  // Step 1: Load the 'filename' parameter
  std::string file;
  pnh.param<std::string>("filename", file, "");

  bool debug_on;
  bool console_debug_on;
  pnh.param<bool>("debug_on", debug_on, false);
  pnh.param<bool>("console_debug_on", console_debug_on, false);

  double vect[3], center[3];
  pnh.param<double>("cut_norm_x", vect[0], 0.0);
  pnh.param<double>("cut_norm_y", vect[1], 0.0);
  pnh.param<double>("cut_norm_z", vect[2], 0.0);
  pnh.param<double>("centroid_x", center[0], 0.0);
  pnh.param<double>("centroid_y", center[1], 0.0);
  pnh.param<double>("centroid_z", center[2], 0.0);

  vtk_viewer::VTK_LOGGER->setLevel(console_debug_on ? log4cxx::Level::getDebug(): log4cxx::Level::getInfo());

  // load tool config
  tool_path_planner::ProcessTool tool = loadTool(pnh);
  tool_path_planner::RasterToolPathPlanner planner;
  planner.enableConsoleDebug(console_debug_on);


  if(!file.empty())
  {
    // read data file
    vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
    std::vector<char> buffer (file.size() + 1, '\0');
    char* str = buffer.data();
    strcpy(str, file.c_str());

    char * pch;
    pch = strtok (str," ,.-");
    while (pch != NULL)
    {
      std::string extension(pch);
      if(extension == "pcd" || extension == "stl" || extension == "STL" || extension == "ply")
      {
        break;
      }
      pch = strtok (NULL, " ,.-");
    }

    std::string extension = std::string(pch);
    if(extension == "pcd")
    {
      if(argc == 3)
      {
        std::string background = argv[2];
        vtk_viewer::loadPCDFile(file, data, background, true);
      }
      else
      {
        vtk_viewer::loadPCDFile(file, data);
      }
    }
    else if(extension == "STL" || extension == "stl")
    {
      data = vtk_viewer::readSTLFile(file);
    }
    else if (toLower(extension) == "ply") // PCL polygon mesh
    {
      pcl::PolygonMesh pcl_mesh;
      vtk_viewer::loadPolygonMeshFromPLY(file, pcl_mesh);
      vtk_viewer::pclEncodeMeshAndNormals(pcl_mesh, data);
    }
    else
    {
      ROS_ERROR("Unrecognized extension: '%s'. Program supports 'pcd', 'stl', 'STL', 'ply'", extension.c_str());
      return 1;
    }

    vtk_viewer::generateNormals(data);

    std::vector<vtkSmartPointer<vtkPolyData> >meshes;
    meshes.push_back(data);

    std::string log_directory = ros::file_log::getLogDirectory();

    planner.setTool(tool);
    planner.setCutDirection(vect);
    planner.setCutCentroid(center);
    planner.setDebugMode(debug_on);
    planner.setLogDir(log_directory);
    std::vector< std::vector<tool_path_planner::ProcessPath> > paths;
    planner.planPaths(meshes, paths);

    // visualize results
    double scale = tool.pt_spacing * 1.5;
    noether::Noether viz;
    viz.setLogDir(log_directory);
    ROS_INFO_STREAM("log directory " << viz.getLogDir());

    viz.addMeshDisplay(meshes);
    viz.addPathDisplay(paths, scale, true, false, false);
    viz.visualizeDisplay();
  }
  else
  {
    ROS_WARN_STREAM("'filename' parameter must be set to the path of a pcd or stl file");
  }


  return 0;
}
