/**
 * @file raster_path_generator.h
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @author Jorge Nicho
 * @date Nov 15, 2018
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_TOOL_PATH_PLANNER_RASTER_PATH_GENERATOR_H_
#define INCLUDE_TOOL_PATH_PLANNER_RASTER_PATH_GENERATOR_H_

#include <shape_msgs/Mesh.h>
#include <boost/optional.hpp>
#include <json/json.h>
#include <pcl/PolygonMesh.h>
#include <vector>
#include <tool_path_planner/path_generator.h>
#include <ros/console.h>

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkKdTreePointLocator.h>
#include <vtkCellLocator.h>
#include <vtkModifiedBSPTree.h>
#include <vtkParametricSpline.h>
#include <vtk_viewer/vtk_viewer.h>
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

class SurfaceWalkRasterGenerator : public PathGenerator
{
  static constexpr double DEFAULT_RASTER_SPACING = 0.04;
  static constexpr double DEFAULT_POINT_SPACING = 0.01;
  static constexpr double DEFAULT_RASTER_ROT_OFFSET = 0.0;
  static constexpr double DEFAULT_MIN_SEGMENT_SIZE = 0.01;
  static constexpr double DEFAULT_MIN_HOLE_SIZE = 1e-2;
  static constexpr double DEFAULT_INTERSECTION_PLANE_HEIGHT = 0;
  static constexpr double DEFAULT_TOOL_OFFSET = 0;
  static constexpr bool DEFAULT_GENERATE_EXTRA_RASTERS = false;
  static constexpr bool DEFAULT_RASTER_WRT_GLOBAL_AXIS = false;
public:

  struct Config
  {
    double raster_spacing {DEFAULT_RASTER_SPACING};
    double point_spacing {DEFAULT_POINT_SPACING};
    double raster_rot_offset {DEFAULT_RASTER_ROT_OFFSET};
    double min_segment_size {DEFAULT_MIN_SEGMENT_SIZE};
    double min_hole_size {DEFAULT_MIN_HOLE_SIZE};
    double intersection_plane_height {DEFAULT_INTERSECTION_PLANE_HEIGHT};
    double tool_offset {DEFAULT_TOOL_OFFSET};
    bool generate_extra_rasters {DEFAULT_GENERATE_EXTRA_RASTERS};
    bool raster_wrt_global_axes {DEFAULT_RASTER_WRT_GLOBAL_AXIS};
    double cut_direction [3] {0, 0, 0};
    double cut_centroid [3] {0, 0, 0};
    bool debug {false};

    Json::Value toJson() const
    {
      Json::Value jv(Json::ValueType::objectValue);
      jv["raster_spacing"] = raster_spacing;
      jv["point_spacing"] = point_spacing;
      jv["raster_rot_offset"] = raster_rot_offset;
      jv["min_segment_size"] = min_segment_size;
      jv["min_hole_size"] = min_hole_size;
      jv["intersection_plane_height"] = intersection_plane_height;
      jv["tool_offset"] = tool_offset;
      jv["generate_extra_rasters"] = generate_extra_rasters;

      return jv;
    }

    bool fromJson(const Json::Value& jv)
    {
      if(jv.isNull())
      {
        ROS_ERROR("Json value is null");
        return false;
      }
      if(jv.type() != Json::ValueType::objectValue)
      {
        ROS_ERROR( "Json type %i is invalid, only '%i' is allowed",static_cast<int>(jv.type()), static_cast<int>(Json::ValueType::objectValue));
        return false;
      }
      auto validate = [](const Json::Value& jv,const std::string& name_, const Json::ValueType& type_) -> bool
      {
        return jv.isMember(name_) && jv[name_].type() == type_;
      };
      raster_spacing = validate(jv,"raster_spacing",Json::ValueType::realValue) ? jv["raster_spacing"].asDouble() : DEFAULT_RASTER_SPACING;
      point_spacing = validate(jv,"point_spacing",Json::ValueType::realValue) ? jv["point_spacing"].asDouble() : DEFAULT_POINT_SPACING;
      raster_rot_offset = validate(jv,"raster_rot_offset", Json::ValueType::realValue) ? jv["raster_rot_offset"].asDouble() : DEFAULT_RASTER_ROT_OFFSET;
      min_segment_size = validate(jv,"min_segment_size", Json::ValueType::realValue) ? jv["min_segment_size"].asDouble() : DEFAULT_MIN_SEGMENT_SIZE;
      min_hole_size = validate(jv,"min_hole_size",Json::ValueType::realValue) ? jv["min_hole_size"].asDouble() : DEFAULT_MIN_HOLE_SIZE;
      intersection_plane_height = validate(jv,"intersection_plane_height",Json::ValueType::realValue) ? jv["intersection_plane_height"].asDouble() : DEFAULT_INTERSECTION_PLANE_HEIGHT;
      tool_offset = validate(jv,"tool_offset",Json::ValueType::realValue) ? jv["tool_offset"].asDouble() : DEFAULT_TOOL_OFFSET;
      generate_extra_rasters = validate(jv,"generate_extra_rasters",Json::ValueType::booleanValue) ? jv["generate_extra_rasters"].asBool() : DEFAULT_GENERATE_EXTRA_RASTERS;
      return true;
      return true;
    }

    bool fromJson(const std::string& jv_string)
    {
      Json::Value jv;
      Json::Reader r;
      if (!r.parse(jv_string, jv))
        return false;

      return fromJson(jv);
    }

    std::string str()
    {
      std::stringstream ss;
      ss<<"raster_spacing: "<< raster_spacing<<std::endl;
      ss<<"point_spacing: "<< point_spacing<<std::endl;
      ss<<"raster_rot_offset: "<< raster_rot_offset<<std::endl;
      ss<<"min_segment_size: "<< min_segment_size<<std::endl;
      ss<<"intersection_plane_height: "<< intersection_plane_height<<std::endl;
      ss<<"tool_offset: "<< tool_offset<<std::endl;
      ss<<"generate_extra_rasters: "<< generate_extra_rasters << std::endl;
      return ss.str();
    }
  };

  SurfaceWalkRasterGenerator() = default;
  virtual ~SurfaceWalkRasterGenerator() = default;

  /**
   * @brief Set the generator configuration
   * @param config The configuration
   * @return True if valid configuration, otherwise false.
   */
  bool setConfiguration(const Config& config);

  void setInput(pcl::PolygonMesh::ConstPtr mesh) override;

  void setInput(vtkSmartPointer<vtkPolyData> mesh) override;

  void setInput(const shape_msgs::Mesh& mesh) override;

  vtkSmartPointer<vtkPolyData> getInput() override;

  boost::optional<ToolPaths> generate() override;

  std::string getName() const override;

protected:
  Config config_;
  std::vector<ProcessPath> paths_;                  /**< series of intersecting lines on the given mesh */
  vtk_viewer::VTKViewer debug_viewer_;              /**< The vtk viewer for displaying debug output */
  vtkSmartPointer<vtkPolyData> mesh_data_;          /**< input mesh to operate on */
  vtkSmartPointer<vtkKdTreePointLocator> kd_tree_;  /**< kd tree for finding nearest neighbor points */
  vtkSmartPointer<vtkCellLocator> cell_locator_;    /** @brief allows locating closest cell */
  vtkSmartPointer<vtkModifiedBSPTree> bsp_tree_;    /** @brief use to perform ray casting on the mesh */

  /**
   * @brief getFirstPath Uses the input mesh, generates the first path by intersecting the mesh with a plane
   * @param path The first path generated
   */
  bool getFirstPath(ProcessPath& path);

  /**
   * @brief getNextPath Creates the next path offset from the current path
   * @param this_path The current path, from which to create an offset path
   * @param next_path The next path returned after calling the function
   * @param dist The distance to offset the next path from the current
   * @param test_self_intersection Disables check to see if new path intersects with any previously generated paths
   * @return True if the next path is successfully created, False if no path can be generated
   */
  bool getNextPath(const ProcessPath this_path, ProcessPath& next_path, double dist = 0.0,
                   bool test_self_intersection = true);

  /**
   * @brief getExtraPath - Using the last path generated, creates and
   * places an extra path that does not need to intersect with the part
   * @param last_path - input - the last valid path
   * @param extra_path - output - a path that may lie beyond the edge of the part
   * @param dist - input - the distance to offset the next path from the current
   * @return True if path is successfully created, False if no path can be generated
   */
  bool getExtraPath(const ProcessPath& last_path, ProcessPath& extra_path, double dist = 0.0);


  /**
   * @brief Estimates the normals of a line that lies on the surface of the current mesh.  For each point, it uses the normal of
   * the closes cell in the mesh
   * @param data  The points to operate on, normal data inserted in place
   * @return True on success, false otherwise.
   */
  bool computeSurfaceLineNormals(vtkSmartPointer<vtkPolyData>& data);

  /**
   * @brief getCellCentroidData Gets the data for a cell in the input_mesh_
   * @param id The cell id to get data for
   * @param center The center of the given cell
   * @param norm The cell normal
   * @param area The cell area
   * @return True if the cell for the given id exists, False if it does not exist
   */
  bool getCellCentroidData(int id, double* center, double* norm, double& area);

  /**
   * @brief createStartCurve Creates a initial "curve" to begin the path planning from
   * @return A set of VTK points and normals which are then used to create a cutting surface
   */
  vtkSmartPointer<vtkPolyData> createStartCurve();

  /**
   * @brief smoothData Takes in a spline and returns a series of evenly spaced points with normals and derivatives
   * @param spline The input spline to operate on
   * @param points The set of evenly spaced points, with normals
   * @param derivatives The set of evenly spaced points, with derivative data inserted into the "normals" position
   */
  void smoothData(vtkSmartPointer<vtkParametricSpline> spline, vtkSmartPointer<vtkPolyData>& points, vtkSmartPointer<vtkPolyData>& derivatives);

  /**
   * @brief generateNormals Generates point normals for a given mesh, normals are located on polydata vertices
   * @param data The mesh to operate on, normal data inserted in place
   */
  //void generateNormals(vtkSmartPointer<vtkPolyData>& data);

  /**
   * @brief createOffsetLine Given a line with normals, generate a new line which is offset by a given distance
   * @param line Start line
   * @param derivatives Derivatives of the start line
   * @param dist The amount and direction to offset
   * @return The newly created line
   */
  vtkSmartPointer<vtkPolyData> createOffsetLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPolyData> derivatives, double dist);

  /**
   * @brief createSurfaceFromSpline Using a line with normals, generate a surface with points above and below the line (used for mesh intersection calculation)
   * @param line The input line with normals
   * @param dist The amount to extend above and below the line for generating the new surface
   * @return The new surface, in the form of a mesh
   */
  vtkSmartPointer<vtkPolyData> createSurfaceFromSpline(vtkSmartPointer<vtkPolyData> line, double dist);

  /**
   * @brief Creates planes that connect the line to its projection on the surface mesh.
   * @param line  The  line
   * @param intersection_dist  Rays queries are created from points in the line along the local normal vector
   *                            by this distance.
   * @return  The surface planes in the form of a mesh.
   */
  vtkSmartPointer<vtkPolyData> extrudeSplineToSurface(vtkSmartPointer<vtkPolyData> line, double intersection_dist);

  /**
   * @brief sortPoints Sorts points in order to form a contiguous line with the shortest length possible
   * @param points The input points to reorder
   */
  void sortPoints(vtkSmartPointer<vtkPoints>& points);

  /**
   * @brief findIntersectionLine Given an cutting mesh, finds the intersection of the mesh and the input_mesh_
   * @param cut_surface The mesh to intersect with the input_mesh_
   * @param points The points found on the intersection of the two meshes
   * @param spline A smoothed spline which is generated from the points found from the intersection
   * @return True if the two meshes intersect, False if they do not (no point or spline data)
   */
  bool findIntersectionLine(vtkSmartPointer<vtkPolyData> cut_surface,
                                                    vtkSmartPointer<vtkPolyData>& points,
                                                    vtkSmartPointer<vtkParametricSpline>& spline);

  /**
   * @brief getConnectedIntersectionLine Given a series of lines, returns a single continuous line while filtering out small segments
   * @param line The input line data (usually obtained from the vtkIntersectionFilter)
   * @param points The list of points in order creating a continuous line
   */
  void getConnectedIntersectionLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPoints>& points);

  /**
   * @brief getConnectedIntersectionLine Given an intersection line data and a start location, finds and returns a list of connected line segments
   * @param line The line data from a vtkIntersectionFilter (or other line data object)
   * @param points The output points which form the continuous line segment
   * @param used_ids The list of ids used in this line segment
   * @param start_pt Optional: the point id in the line to start from (useful when performing this operation multiple times)
   * @return the length of the line segment returned
   */
  double getConnectedIntersectionLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPoints>& points, std::vector<int> &used_ids, int start_pt = 0);

  /**
   * @brief checkPathForHoles Checks a given path to determine if it needs to be broken up if there is a large hole in the middle
   * @param path The input path to be checked for large holes/gaps
   * @param out_paths The output paths, after any splitting is performed.  Is empty if no splitting is needed
   * @return True if a large hole is detected and path was broken up, false if no splitting is needed
   */
  bool checkPathForHoles(const ProcessPath path, std::vector<ProcessPath>& out_paths);

  /**
   * @brief resamplePoints Resamples a set of points to make them evenly spaced, creates and samples a spline through the original point set
   * @param points The input points to modify
   */
  void resamplePoints(vtkSmartPointer<vtkPoints>& points);
};

} /* namespace tool_path_planner */

#endif /* INCLUDE_TOOL_PATH_PLANNER_RASTER_PATH_GENERATOR_H_ */
