#ifndef TOOL_PATH_PLANNER_PATH_GENERATOR_H
#define TOOL_PATH_PLANNER_PATH_GENERATOR_H

#include <boost/optional.hpp>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkParametricSpline.h>
#include <vtkPolyData.h>

namespace tool_path_planner
{
using ToolPathSegment = std::vector<Eigen::Isometry3d>;
using ToolPath = std::vector<ToolPathSegment>;
using ToolPaths = std::vector<ToolPath>;

struct ToolPathSegmentData
{
  /** @brief sequence of points and a normal defining the locations and z-axis orientation of the tool along the path */
  vtkSmartPointer<vtkPolyData> line;
  /** @brief spline used to generate the line lamda goes from 0 to 1 as the line goes from start to finish */
  vtkSmartPointer<vtkParametricSpline> spline;
  /** @brief derivatives are the direction of motion along the spline */
  vtkSmartPointer<vtkPolyData> derivatives;
};
using ToolPathData = std::vector<ToolPathSegmentData>;
using ToolPathsData = std::vector<ToolPathData>;

class PathGenerator
{
public:
  using Ptr = std::shared_ptr<PathGenerator>;
  using ConstPtr = std::shared_ptr<const PathGenerator>;

  PathGenerator() = default;
  virtual ~PathGenerator() = default;
  PathGenerator(const PathGenerator&) = delete;
  PathGenerator& operator=(const PathGenerator&) = delete;
  PathGenerator(PathGenerator&&) = delete;
  PathGenerator& operator=(PathGenerator&&) = delete;

  /**
   * @brief sets the input mesh from which raster paths are to be generated
   * @param mesh The mesh input
   */
  virtual void setInput(pcl::PolygonMesh::ConstPtr mesh) = 0;

  /**
   * @brief sets the input mesh from which raster paths are to be generated
   * @param mesh The mesh input
   */
  virtual void setInput(vtkSmartPointer<vtkPolyData> mesh) = 0;

  /**
   * @brief sets the input mesh from which raster paths are to be generated
   * @param mesh The mesh input
   */
  virtual void setInput(const shape_msgs::Mesh& mesh) = 0;

  /**
   * @brief Get the input mesh
   * @return The input mesh
   */
  virtual vtkSmartPointer<vtkPolyData> getInput() = 0;

  /**
   * @brief Generate the raster paths that follow the contour of the mesh
   * @param config The configuration
   * @return  An array of tool paths or boost::none when it fails.
   */
  virtual boost::optional<ToolPaths> generate() = 0;

  /**
   * @brief the class name
   * @return a string
   */
  virtual std::string getName() const = 0;
};
}  // namespace tool_path_planner
#endif  // TOOL_PATH_PLANNER_PATH_GENERATOR_H
