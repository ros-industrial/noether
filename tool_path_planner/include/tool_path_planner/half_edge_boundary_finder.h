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

#ifndef INCLUDE_TOOL_PATH_PLANNER_HALF_EDGE_BOUNDARY_FINDER_H_
#define INCLUDE_TOOL_PATH_PLANNER_HALF_EDGE_BOUNDARY_FINDER_H_

#include <boost/optional.hpp>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/PoseArray.h>

namespace tool_path_planner
{


/**
 * @class tool_path_planner::HalfEdgeBoundaryFinder
 * @details Computes the edges of a mesh by extracting the mesh half edges, needs a mesh that does not have duplicate points
 */
class HalfEdgeBoundaryFinder
{
public:

  struct Config
  {
    std::size_t min_num_points = 200;   /**@brief only edge segments with more than this many points will be returned*/
    double min_point_dist = 0.01;       /**@brief minimum distance for adjacent points, set to < 0 to turn off this constraint */
  };

  HalfEdgeBoundaryFinder();
  virtual ~HalfEdgeBoundaryFinder();


  /**
   * @brief sets the input mesh from which edges are to be generated
   * @param mesh The mesh input
   */
  void setInput(pcl::PolygonMesh::ConstPtr mesh);

  /**
   * @brief sets the input mesh from which edges are to be generated
   * @param mesh The mesh input
   */
  void setInput(const shape_msgs::Mesh& mesh);

  /**
   * @brief Generate the edge poses that follow the contour of the mesh
   * @param config The configuration
   * @return  An array of edge poses or boost::none when it fails.
   */
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(const HalfEdgeBoundaryFinder::Config& config);

protected:
  pcl::PolygonMesh::ConstPtr mesh_;
};

} /* namespace tool_path_planner */

#endif /* INCLUDE_TOOL_PATH_PLANNER_HALF_EDGE_BOUNDARY_FINDER_H_ */
