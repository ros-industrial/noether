/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
    int simulator_nearest_neighbors;            /** how many neighbors are used to compute local normals*/
    double min_hole_size;             /** A path may pass over holes smaller than this, but must be broken when larger holes
                                          are encountered */
    double min_segment_size;          /** The minimum segment size to allow when finding intersections; small segments will
                                          be discarded */
     /** @brief Specifies the direction of the rasters wrt either the mesh coordinates or the principal axis. Rotation is in radians. Default is 0.0*/
    double  raster_angle;

    /** @brief Specifies axis about which raster_angle_ is applied.
     *
     * If false, raster angle is specified about the smallest axis of the
     * bounding box with 0 being in the direction of the principal axis.
     *
     * If true, the raster angle is about the mesh z coordinate with the x
     * axis being 0. Then the resultant vector is projected onto the plane
     * created by the bounding box x and y axes */
    bool raster_wrt_global_axes;
    double simulator_tool_radius; /** the radius of the tool (for process simulation)*/
    double simulator_tool_height; /** the height of the tool (for process simulation)*/
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
