/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#ifndef NOETHER_H
#define NOETHER_H

#include <vtk_viewer/vtk_viewer.h>
#include <mesh_segmenter/mesh_segmenter.h>
#include <tool_path_planner/tool_path_planner.h>
#include <path_sequence_planner/path_sequence_planner.h>

namespace noether
{

  class Noether
  {

  public:
    Noether(){}
    ~Noether(){}

    /**
     * @brief addMeshDisplay Add a display for a vector of meshes
     * @param meshes The vector to be displayed
     */
    void addMeshDisplay(std::vector<vtkSmartPointer<vtkPolyData> >& meshes);

    /**
     * @brief addPathDisplay  Add a display for a vector of tool paths
     * @param paths
     * @param show_path
     * @param show_cutting_meshes
     * @param show_derivatives
     */
    void addPathDisplay(std::vector<std::vector< tool_path_planner::ProcessPath > >& paths,
                        bool show_path = true, bool show_cutting_meshes = false, bool show_derivatives = false);

    /**
     * @brief addPathDisplay  Add a display for a vector of tool paths
     * @param paths
     * @param show_path
     * @param show_cutting_meshes
     * @param show_derivatives
     */
    void addPathDisplay(std::vector< tool_path_planner::ProcessPath >& paths, bool show_path = true,
                        bool show_cutting_meshes = false, bool show_derivatives = false);

    void visualizeDisplay(){viewer_.renderDisplay();}

  private:
    vtk_viewer::VTKViewer viewer_;
  };
}

#endif // NOETHER_H
