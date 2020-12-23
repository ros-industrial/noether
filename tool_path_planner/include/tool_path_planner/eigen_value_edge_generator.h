/**
 * @author Jorge Nicho
 * @file edge_path_generator.h
 * @date Nov 9, 2019
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

#ifndef INCLUDE_TOOL_PATH_PLANNER_EIGEN_VALUE_EDGE_GENERATOR_H_
#define INCLUDE_TOOL_PATH_PLANNER_EIGEN_VALUE_EDGE_GENERATOR_H_

#include <boost/optional.hpp>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/octree/octree_search.h>
#include <tool_path_planner/path_generator.h>

namespace tool_path_planner
{
/**
 * @brief The EigenValueEdgeGenerator class
 *
 * @details Currently this approach use the the geometric feature 'Surface Variation' to identify edges/contour which
 * does not work well. Need to explore the other feature below to see if they produce better results. This feature is
 * go at detecting creases but not the contour.
 *
 * The information was pulled from the paper "Contour detection in unstructured 3D point cloud" by Timo Hackel,
 * Jan D. Wegner, Konrad Schindler.
 *
 * Geometric features based on eigen values:
 *   - Sum                 = sigma1 + sigma2 + sigma3
 *   - Omnivariance        = (sigma1 + sigma2 + sigma3)^(1/3)
 *   - Eigenentropy        = -(sigma1*ln(sigma1) + sigma2*ln(sigma2) + sigma3*ln(sigma3))
 *   - Anisotropy          = (sigma1 - sigma3) / sigma1
 *   - Planarity           = (sigma2 - sigma3) / sigma1
 *   - Linearity           = (sigma1 - sigma2) / sigma1
 *   - Surface Variation   = sigma3 / (sigma1 + sigma2 + sigma3)
 *   - Sphericity          = sigma3 /sigma1
 *   - First Order Moment  = See Paper Eq. 2
 *   - Line Feature        = See Paper Eq. 3
 *   - Orientation Feature = See Paper Eq. 4
 *   - Verticality         = See Paper
 */
class EigenValueEdgeGenerator : public PathGenerator
{
public:
  struct Config
  {
    /**
     * @ingroup  hsv values computation using the covariance matrix
     * @{
     */
    double octree_res = 0.005;   /** @brief resolution for the octree */
    double search_radius = 0.01; /** @brief radious used for grouping */
    std::size_t num_threads = 4; /** @brief desired omp thread count  */
    /** @}*/

    /**
     * @ingroup edge filtering
     * @{
     */
    double neighbor_tol = 0.01; /** @brief points with "v > max_v * (1 - neighbor_tol)" are considered edge points  */
    /** @}*/

    /**
     * @ingroup  edge point reordering
     * @{
     */
    double voxel_size = 0.01;     /** @brief used in downsampling edge points  */
    int edge_cluster_min = 3;     /** @brief downsamples only when edge cluster size is greater than this value  */
    double kdtree_epsilon = 0.01; /** @brief precision used for nearest neighbor searches  */
    /** @}*/

    /**
     * @ingroup edge segment splitting
     * @brief uses ray-casting against the mesh's octree minus the edge points.
     * @{
     */
    double min_projection_dist = 0.01; /** @brief minimum distance by which to project an edge point along the normal,
                                                  recommended 2 * octree_res */
    int max_intersecting_voxels = 4; /** @brief will split when distance exceeds octree_res * max_intersecting_voxels */
    /** @}*/

    double merge_dist =
        0.01; /** @brief any two consecutive points with a shortest distance smaller than this value are merged */

    double max_segment_length = 1.0; /** @brief maximum segment length */
  };

  EigenValueEdgeGenerator() = default;

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
  /** @brief Setup Helper Function */
  void setup();

  /**
   * @brief splits the edge into multiple segments that are separated by surfaces
   * @param config        The configuration
   * @param edge_indices  The indices of the "ordered" points at the edges
   * @param segments      The output indices of each edge segment
   * @return  True on success, false otherwise
   */
  bool splitEdgeSegments(const pcl::PointIndices& edge_indices, std::vector<pcl::PointIndices>& segments);

  /**
   * @brief merge points that are very close to one another
   * @param config          Contains the threshold value used to decide whether two consecutive points should be merged
   * @param edge_segment    The point indices of an edge
   * @param merged_points   The merged points
   * @return  Number of points that were merged.
   */
  int mergePoints(const pcl::PointIndices& edge_segment, pcl::PointCloud<pcl::PointNormal>& merged_points);

  /**
   * @brief Checks if the there's a surface that separates two points
   * @param config  The configuration
   * @param p1      The start point
   * @param p2      The end point
   * @return  The number of surface voxels that were encountered
   */
  int checkSurfaceIntersection(const pcl::PointNormal& p1, const pcl::PointNormal& p2);

  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_points_;
  pcl::PolygonMesh::ConstPtr mesh_;
  vtkSmartPointer<vtkPolyData> vtk_mesh_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_search_;
  Config config_;
};

} /* namespace tool_path_planner */

#endif /* INCLUDE_TOOL_PATH_PLANNER_EIGEN_VALUE_EDGE_GENERATOR_H_ */
