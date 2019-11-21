/*
 * edge_path_generator.h
 *
 *  Created on: Nov 9, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_TOOL_PATH_PLANNER_EDGE_PATH_GENERATOR_H_
#define INCLUDE_TOOL_PATH_PLANNER_EDGE_PATH_GENERATOR_H_

#include <boost/optional.hpp>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/octree/octree_search.h>

namespace tool_path_planner
{

struct EdgePathConfig
{
  /**
   * @ingroup  hsv values computation using the covariance matrix
   * @{
   */
  double octree_res = 0.005;    /** @brief resolution for the octree */
  double search_radius = 0.01;  /** @brief radious used for grouping */
  std::size_t num_threads = 4;  /** @brief desired omp thread count  */
  /** @}*/

  /**
   * @ingroup edge filtering
   * @{
   */
  double neighbor_tol = 0.01;   /** @brief points with "v > max_v * (1 - neighbor_tol)" are considered edge points  */
  /** @}*/

  /**
   * @ingroup  edge point reordering
   * @{
   */
  double voxel_size = 0.01;        /** @brief used in downsampling edge points  */
  int edge_cluster_min = 3;        /** @brief downsamples only when edge cluster size is greater than this value  */
  double kdtree_epsilon = 0.01;    /** @brief precision used for nearest neighbor searches  */
  /** @}*/

  /**
   * @ingroup edge segment splitting
   * @brief uses ray-casting against the mesh's octree minus the edge points.
   * @{
   */
  double min_projection_dist = 0.01;  /** @brief minimum distance by which to project an edge point along the normal,
                                                 recommended 2 * octree_res */
  int max_intersecting_voxels = 4;    /** @brief A surface is considered to be between two waypoints when the ray intersects
                                                 a number of voxels greater than this number*/
  /** @}*/

  double merge_dist = 0.01;    /** @brief any two consecutive points with a shortest distance smaller than this value are merged */
};

class EdgePathGenerator
{
public:
  EdgePathGenerator();
  virtual ~EdgePathGenerator();

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
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(const tool_path_planner::EdgePathConfig& config);

  /**
   * @brief Generate the edge poses that follow the contour of the mesh
   * @param mesh  The input mesh from which edges will be generated
   * @param config The configuration
   * @return  An array of edge poses or boost::none when it fails.
   */
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(const shape_msgs::Mesh& mesh,
                                                                    const tool_path_planner::EdgePathConfig& config);

  /**
   * @brief Generate the edge poses that follow the contour of the mesh
   * @param mesh  The input mesh from which edges will be generated
   * @param config The configuration
   * @return  An array of edge poses or boost::none when it fails.
   */
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(pcl::PolygonMesh::ConstPtr mesh,
                                                                    const tool_path_planner::EdgePathConfig& config);


protected:

  /**
   * @brief splits the edge into multiple segments that are separated by surfaces
   * @param config        The configuration
   * @param edge_indices  The indices of the "ordered" points at the edges
   * @param segments      The output indices of each edge segment
   * @return  True on success, false otherwise
   */
  bool splitEdgeSegments(const tool_path_planner::EdgePathConfig& config, const pcl::PointIndices& edge_indices,
                         std::vector<pcl::PointIndices>& segments);


  /**
   * @brief merge points that are very close to one another
   * @param config          Contains the threshold value used to decide whether two consecutive points should be merged
   * @param edge_segment    The point indices of an edge
   * @param merged_points   The merged points
   * @return  Number of points that were merged.
   */
  int mergePoints(const tool_path_planner::EdgePathConfig& config,const pcl::PointIndices& edge_segment,
                                      pcl::PointCloud<pcl::PointNormal>& merged_points);

  /**
   * @brief Checks if the there's a surface that separates two points
   * @param config  The configuration
   * @param p1      The start point
   * @param p2      The end point
   * @return  The number of surface voxels that were encountered
   */
  int checkSurfaceIntersection(const tool_path_planner::EdgePathConfig& config,
                               const pcl::PointNormal& p1, const pcl::PointNormal& p2);

  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_points_;
  pcl::PolygonMesh::ConstPtr mesh_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_search_;


};

} /* namespace tool_path_planner */

#endif /* INCLUDE_TOOL_PATH_PLANNER_EDGE_PATH_GENERATOR_H_ */
