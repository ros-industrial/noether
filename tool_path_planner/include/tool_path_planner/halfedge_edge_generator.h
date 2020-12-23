/**
 * @author Jorge Nicho
 * @file mesh_boundary_finder.h
 * @date Dec 5, 2019
 * @copyright Copyright (c) 2019, Southwest Research Institute
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

#ifndef INCLUDE_TOOL_PATH_PLANNER_HALFEDGE_EDGE_GENERATOR_H_
#define INCLUDE_TOOL_PATH_PLANNER_HALFEDGE_EDGE_GENERATOR_H_

#include <boost/optional.hpp>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <tool_path_planner/path_generator.h>

namespace tool_path_planner
{
/**
 * @class tool_path_planner::HalfedgeEdgeGenerator
 * @details Computes the edges of a mesh by extracting the mesh half edges, needs a mesh that does not have duplicate
 * points
 */
class HalfedgeEdgeGenerator : public PathGenerator
{
public:
  enum PointSpacingMethod : int
  {
    NONE = 0,
    MIN_DISTANCE = 1,
    EQUAL_SPACING = 2,
    PARAMETRIC_SPLINE = 3
  };

  struct Config
  {
    /**@brief only edge segments with more than this many points will be returned*/
    std::size_t min_num_points = 200;

    /**@brief True in order set the normal of each point as the average of the normal vectors of the points within a
     * specified radius*/
    bool normal_averaging = true;

    /**@brief The search radius used for normal averaging */
    double normal_search_radius = 0.02;

    /**@brief A value [0, 1] that influences the normal averaged based on its distance, set to 0 to disable */
    double normal_influence_weight = 0.5;

    /**@brief the method used for spacing the points, NONE = 0, MIN_DISTANCE = 1, EQUAL_SPACING = 2, PARAMETRIC_SPLINE =
     * 3 */
    PointSpacingMethod point_spacing_method = PointSpacingMethod::EQUAL_SPACING;

    /**@brief point distance parameter used in conjunction with the spacing method */
    double point_dist = 0.01;

    /** @brief maximum segment length */
    double max_segment_length = 1.0;
  };

  HalfedgeEdgeGenerator() = default;

  /**
   * @brief Set the generator configuration
   * @param config The configuration
   * @return True if valid configuration, otherwise false.
   */
  void setConfiguration(const Config& config);

  void setInput(pcl::PolygonMesh::ConstPtr mesh) override;

  void setInput(vtkSmartPointer<vtkPolyData> mesh) override;

  void setInput(const shape_msgs::Mesh& mesh) override;

  vtkSmartPointer<vtkPolyData> getInput() override;

  boost::optional<ToolPaths> generate() override;

  std::string getName() const override;

protected:
  pcl::PolygonMesh::ConstPtr mesh_;
  vtkSmartPointer<vtkPolyData> vtk_mesh_;
  Config config_;
};

} /* namespace tool_path_planner */

#endif /* INCLUDE_TOOL_PATH_PLANNER_HALFEDGE_EDGE_GENERATOR_H_ */
