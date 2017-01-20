/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#ifndef TOOL_PATH_PLANNER_H
#define TOOL_PATH_PLANNER_H

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkParametricSpline.h>
#include <vtkKdTreePointLocator.h>

#include <vtk_viewer/vtk_viewer.h>

namespace tool_path_planner
{
  struct ProcessPath
  {
    vtkSmartPointer<vtkPolyData> line;
    vtkSmartPointer<vtkParametricSpline> spline;
    vtkSmartPointer<vtkPolyData> derivatives;
    vtkSmartPointer<vtkPolyData> intersection_plane;
  };

  struct ProcessTool
  {
    double pt_spacing;
    double line_spacing;
    double tool_offset;
    double intersecting_plane_height;
    int nearest_neighbors;
    double min_hole_size;
  };

  class ToolPathPlanner
  {
  public:

    //ToolPathPlanner()
    virtual ~ToolPathPlanner(){}

    /**
     * @brief planPaths plans a set of paths for all meshes in a given list
     * @param meshes A vector of meshes to plan paths for
     * @param paths The resulting path data generated
     */
    virtual void planPaths(std::vector<vtkSmartPointer<vtkPolyData> > meshes, std::vector< std::vector<ProcessPath> >& paths)=0;

    /**
     * @brief setInputMesh Sets the input mesh to generate paths
     * @param mesh The input mesh to be operated on
     */
    virtual void setInputMesh(vtkSmartPointer<vtkPolyData> mesh)=0;

    /**
     * @brief getInputMesh Gets the input mesh used for generating paths
     * @return The stored input mesh
     */
    virtual vtkSmartPointer<vtkPolyData> getInputMesh()=0;

    /**
     * @brief setTool Sets the tool parameters used during path generation
     * @param tool The tool object with all of the parameters necessary for path generation
     */
    virtual void setTool(ProcessTool tool)=0;

    /**
     * @brief getTool Gets the tool parameters used during path generation
     * @return The set of tool parameters
     */
    virtual ProcessTool getTool()=0;

    /**
     * @brief getNextPath Creates the next path offset from the current path
     * @param this_path The current path, from which to create an offset path
     * @param next_path The next path returned after calling the function
     * @param dist The distance to offset the next path from the current
     * @return True if the next path is successfully created, False if no path can be generated
     */
    virtual bool getNextPath(const ProcessPath this_path, ProcessPath& next_path, double dist = 0.0)=0;

    /**
     * @brief computePaths Will create and store all paths possible from the given mesh and starting path
     * @return True if paths were generated, False if the first path is not available (nothing to start from)
     */
    virtual bool computePaths()=0;

    /**
     * @brief getPaths Gets all of the paths generated
     * @return The paths generated from the computePaths() function
     */
    virtual std::vector<ProcessPath> getPaths()=0;

    /**
     * @brief setDebugModeOn Turn on debug mode to visualize every step of the path planning process
     * @param debug Turns on debug if true, turns off debug if false
     */
    virtual void setDebugMode(bool debug)=0;
  };

}

#endif // PATH_PLANNER_H
