/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "noether/noether.h"
#include <vtkPointData.h>

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

int main(int argc, char **argv)
{
  if(argc >= 2)
  {
    // read data file
    vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
    std::string file = argv[1];

    char str[100];
    strcpy(str, file.c_str());

    char * pch;
    pch = strtok (str," ,.-");
    while (pch != NULL)
    {
      std::string extension(pch);
      if(extension == "pcd" || extension == "stl" || extension == "STL")
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
    else
    {
      return 1;
    }

    vtk_viewer::generateNormals(data);

    // segment mesh
    //mesh_segmenter::MeshSegmenter seg;
    //seg.setInputMesh(data);
    //seg.segmentMesh();
    std::vector<vtkSmartPointer<vtkPolyData> > meshes;
    meshes.push_back(data);

    // plan paths for segmented meshes
    tool_path_planner::RasterToolPathPlanner planner;
    tool_path_planner::ProcessTool tool;
    tool.pt_spacing = 0.001;
    tool.line_spacing = 0.03;
    tool.tool_offset = 0.0; // currently unused
    tool.intersecting_plane_height = 0.01; // 0.5 works best, not sure if this should be included in the tool
    tool.nearest_neighbors = 5; // not sure if this should be a part of the tool
    planner.setTool(tool);
    std::vector< std::vector<tool_path_planner::ProcessPath> > paths;
    planner.planPaths(meshes, paths);

    // visualize results
    double scale = 0.03;
    noether::Noether viz;
    viz.addMeshDisplay(meshes);
    viz.addPathDisplay(paths, scale, true, false, false);
    viz.visualizeDisplay();
  }


  return 0;
}
