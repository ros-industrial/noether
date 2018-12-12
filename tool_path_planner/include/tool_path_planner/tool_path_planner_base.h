/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#ifndef TOOL_PATH_PLANNER_H
#define TOOL_PATH_PLANNER_H

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkParametricSpline.h>

#include <pcl/PolygonMesh.h>

namespace tool_path_planner
{
  struct ProcessPath
  {
    vtkSmartPointer<vtkPolyData> line; // sequence of points and a normal defining the locations and z-axis orientation of the tool along the path
    vtkSmartPointer<vtkParametricSpline> spline; // spline used to generate the line lamda goes from 0 to 1 as the line goes from start to finish
    vtkSmartPointer<vtkPolyData> derivatives; // derivatives are the direction of motion along the spline
    vtkSmartPointer<vtkPolyData> intersection_plane; // May belong here, ok to return empty{}, used by the raster_tool_path_planner and returned for display
  };

  struct ProcessTool
  {
    double pt_spacing;                /** required spacing between path points */
    double line_spacing;              /** offset between two rasters  */
    double tool_offset;               /** how far off the surface the tool needs to be */
    double intersecting_plane_height; /** Used in creating planes from that originate from an adjacent raster line to
                                          the surface*/
    double min_hole_size;             /** A path may pass over holes smaller than this, but must be broken when larger holes
                                          are encountered */
    double min_segment_size;          /** The minimum segment size to allow when finding intersections; small segments will
                                          be discarded */
  };

  /**
   * @class tool_path_planner::ToolPathPlanner
   * @brief Interface class for tool path planner implementations.
   */
  class ToolPathPlannerBase
  {
  public:

    virtual ~ToolPathPlannerBase(){}

    /**
     * @brief planPaths plans a set of paths for all meshes in a given list
     * @param meshes A vector of meshes to plan paths for
     * @param paths The resulting path data generated
     */
    virtual void planPaths(const vtkSmartPointer<vtkPolyData> mesh, std::vector<ProcessPath>& paths)=0;
    virtual void planPaths(const std::vector<vtkSmartPointer<vtkPolyData> > meshes, std::vector< std::vector<ProcessPath> >& paths)=0;
    virtual void planPaths(const std::vector<pcl::PolygonMesh>& meshes, std::vector< std::vector<ProcessPath> >& paths)=0;
    virtual void planPaths(const pcl::PolygonMesh& mesh, std::vector<ProcessPath>& paths)=0;

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
