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
#include <pcl/octree/octree_search.h>

namespace tool_path_planner
{

struct EdgePathConfig
{
  // compute hsv values
  double octree_res = 0.005;
  double search_radius = 0.01;
  std::size_t num_threads = 4;

  // edge filtering
  double neighbor_tol = 0.01;

  // edge segment clustering
  double cluster_tol = 0.02;
  int cluster_min = 3;
  int cluster_max = 1000;

  // edge point reordering
  double voxel_size = 0.01;
  double kdtree_epsilon = 0.01;
};

class EdgePathGenerator
{
public:
  EdgePathGenerator();
  virtual ~EdgePathGenerator();

  void setInput(pcl::PolygonMesh::ConstPtr mesh, double octree_res = 0.01);
  void setInput(const shape_msgs::Mesh& mesh, double octree_res = 0.01);

  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(const tool_path_planner::EdgePathConfig& config);
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(const shape_msgs::Mesh& mesh,
                                                                    const tool_path_planner::EdgePathConfig& config);
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(pcl::PolygonMesh::ConstPtr mesh,
                                                                    const tool_path_planner::EdgePathConfig& config);


protected:

  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud_;
  pcl::PolygonMesh::ConstPtr mesh_;
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr edge_results_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_search_;


};

} /* namespace tool_path_planner */

#endif /* INCLUDE_TOOL_PATH_PLANNER_EDGE_PATH_GENERATOR_H_ */
