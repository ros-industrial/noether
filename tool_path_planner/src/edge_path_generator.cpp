/*
 * edge_path_generator.cpp
 *
 *  Created on: Nov 9, 2019
 *      Author: jrgnicho
 */

#include <boost/make_shared.hpp>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <noether_conversions/noether_conversions.h>
#include <console_bridge/console.h>
#include <eigen_conversions/eigen_msg.h>
#include "tool_path_planner/edge_path_generator.h"


bool filterEdges(pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr input, pcl::PointIndices& indices, double percent_threshold = 0.05)
{
  using namespace pcl;
  using PType = std::remove_reference<decltype(*input)>::type::PointType;
  using CondType = pcl::ConditionAnd<PType>;

  double min, max, span;
  // ======================= Determine max in v channel ==========================
  max = (*std::max_element(input->begin(),input->end(),[](PType p1, PType p2){
    return p1.v < p2.v;
  })).v;
  span = max * (percent_threshold);
  min = max - 2 * span;
  CONSOLE_BRIDGE_logInform("Found v channel min and max: (%f, %f)",min, max);

  // ======================= Filtering v channel ============================

  // build the range condition
  CondType::Ptr range_cond  = boost::make_shared<CondType>();
  range_cond->addComparison(boost::make_shared< FieldComparison<PType> >("v", ComparisonOps::LT,min));
  //range_cond->addComparison(boost::make_shared< FieldComparison<PType> >("v", ComparisonOps::GT,max));

  // build the filter
  ConditionalRemoval<PType> cond_filter(true);
  cond_filter.setInputCloud(input);
  cond_filter.setCondition(range_cond);
  cond_filter.setKeepOrganized(false);

  pcl::PointCloud<PType> filtered_cloud;
  cond_filter.filter(filtered_cloud);

  cond_filter.getRemovedIndices(indices);
  CONSOLE_BRIDGE_logInform("Retained %lu points out of %lu",indices.indices.size(),input->size());
  return !indices.indices.empty();
}

bool reorder(const tool_path_planner::EdgePathConfig& config,
             pcl::PointCloud<pcl::PointXYZ>::ConstPtr points ,const pcl::PointIndices& cluster_indices,
             std::vector<int>& rearranged_indices)
{
  using namespace pcl;
  const int k_points = 1;
  pcl::IndicesPtr active_indices = boost::make_shared<decltype(cluster_indices.indices)>(cluster_indices.indices);

  // downsampling first
  if(cluster_indices.indices.size() > config.cluster_min)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled = boost::make_shared<PointCloud<PointXYZ>>();
    pcl::VoxelGrid<PointXYZ> voxelgrid;
    voxelgrid.setInputCloud(points);
    voxelgrid.setIndices(active_indices);
    voxelgrid.setLeafSize(config.voxel_size, config.voxel_size, config.voxel_size);
    voxelgrid.filter(*cloud_downsampled);
    CONSOLE_BRIDGE_logInform("Downsampled edge cluster to %lu points from %lu", cloud_downsampled->size(), active_indices->size());

    // using kd tree to find closest points from original cloud
    std::vector<int> retained_ind;
    pcl::KdTreeFLANN<pcl::PointXYZ> recover_indices_kdtree;
    recover_indices_kdtree.setEpsilon(config.voxel_size);
    recover_indices_kdtree.setInputCloud(points,active_indices);
    for(std::size_t i = 0; i < cloud_downsampled->size(); i++)
    {
      // find closest
      std::vector<int> k_indices(k_points);
      std::vector<float> k_sqr_distances(k_points);
      PointXYZ query_point = (*cloud_downsampled)[i];
      if(recover_indices_kdtree.nearestKSearch(query_point,k_points, k_indices, k_sqr_distances) < k_points)
      {
        CONSOLE_BRIDGE_logError("Nearest K Search failed to find a point during indices recovery stage");
        return false;
      }
      retained_ind.push_back(k_indices[0]);
    }

    // now assign to active points
    active_indices->assign(retained_ind.begin(), retained_ind.end());
    CONSOLE_BRIDGE_logInform("Retained %lu unique edge cluster points ",active_indices->size());
  }

  // removing repeated points
  std::sort(active_indices->begin(), active_indices->end());
  std::remove_reference< decltype(*active_indices) >::type::iterator last_iter = std::unique(active_indices->begin(), active_indices->end());
  active_indices->erase(last_iter, active_indices->end());

  // build reordering kd tree
  pcl::KdTreeFLANN<pcl::PointXYZ> reorder_kdtree;
  reorder_kdtree.setEpsilon(config.kdtree_epsilon);

  // now reorder based on proximity
  auto& cloud = *points;
  std::vector<int> visited_indices;
  visited_indices.reserve(active_indices->size());
  int search_point_idx = active_indices->front();
  visited_indices.push_back(search_point_idx); // insert first point

  const int max_iters = active_indices->size();
  PointXYZ search_point = cloud[search_point_idx];
  PointXYZ start_point = search_point;
  PointXYZ closest_point;
  int iter_count = 0;

  while(iter_count <= max_iters)
  {
    iter_count++;

    // remove from active
    active_indices->erase(std::remove(active_indices->begin(), active_indices->end(),search_point_idx));

    if(active_indices->empty())
    {
      break;
    }

    // set tree inputs;
    reorder_kdtree.setInputCloud(points,active_indices);

    // find next point
    std::vector<int> k_indices(k_points);
    std::vector<float> k_sqr_distances(k_points);
    int points_found = reorder_kdtree.nearestKSearch(search_point,k_points, k_indices, k_sqr_distances);
    if(points_found < k_points)
    {
      CONSOLE_BRIDGE_logError("Nearest K Search failed to find a point during reordering stage");
      return false;
    }

    // insert new point if it has not been visited
    if(std::find(visited_indices.begin(), visited_indices.end(),k_indices[0]) != visited_indices.end())
    {
      // there should be no more points to add
      CONSOLE_BRIDGE_logWarn("Found repeated point during reordering stage, should not happen but proceeding");
      continue;
    }

    // check if found point is further away than the start point
    closest_point = cloud[k_indices[0]];
    Eigen::Vector3d diff = (start_point.getArray3fMap() - closest_point.getArray3fMap()).cast<double>();
    if(diff.norm() < std::sqrt(k_sqr_distances[0]))
    {
      // reversing will allow adding point from the other end
      std::reverse(visited_indices.begin(),visited_indices.end());
      start_point = cloud[visited_indices[0]];
    }

    // set next search point variables
    search_point_idx = k_indices[0];
    search_point = closest_point;

    // add to visited
    visited_indices.push_back(k_indices[0]);
  }

  if(visited_indices.size() < max_iters)
  {
    CONSOLE_BRIDGE_logError("Failed to include all points in the downsampled group, only got %lu out of %lu",
                            visited_indices.size(), max_iters);
    return false;
  }
  std::copy(visited_indices.begin(), visited_indices.end(),std::back_inserter(rearranged_indices));
  return true;
}

bool createPoseArray(const pcl::PointCloud<pcl::PointNormal>& cloud_normals, const std::vector<int>& indices, geometry_msgs::PoseArray& poses)
{
  using namespace pcl;
  using namespace Eigen;

  geometry_msgs::Pose pose_msg;
  Isometry3d pose;
  Vector3d x_dir, z_dir, y_dir;

  auto point_to_vec = [](const PointNormal& p) -> Vector3d
  {
    return Vector3d(p.x, p.y, p.z);
  };

  auto to_rotation_mat = [](const Vector3d& vx, const Vector3d& vy, const Vector3d& vz) -> Matrix3d
  {
    Matrix3d rot;
    rot.block(0,0,1,3) = Vector3d(vx.x(), vy.x(), vz.x()).array().transpose();
    rot.block(1,0,1,3) = Vector3d(vx.y(), vy.y(), vz.y()).array().transpose();
    rot.block(2,0,1,3) = Vector3d(vx.z(), vy.z(), vz.z()).array().transpose();
    return rot;
  };

  for(std::size_t i = 0; i < indices.size() - 1; i++)
  {
    std::size_t idx_current = indices[i];
    std::size_t idx_next = indices[i+1];
    if(idx_current >= cloud_normals.size() || idx_next >= cloud_normals.size())
    {
      CONSOLE_BRIDGE_logError("Invalid indices (current: %lu, next: %lu) for point cloud were passed",
                              idx_current, idx_next);
      return false;
    }
    const PointNormal& p1 = cloud_normals[idx_current];
    const PointNormal& p2 = cloud_normals[idx_next];
    x_dir = (point_to_vec(p2) - point_to_vec(p1)).normalized().cast<double>();
    z_dir = Vector3d(p1.normal_x, p1.normal_y, p1.normal_z).normalized();
    y_dir = z_dir.cross(x_dir).normalized();

    pose = Translation3d(point_to_vec(p1));
    pose.matrix().block<3,3>(0,0) = to_rotation_mat(x_dir, y_dir, z_dir);
    tf::poseEigenToMsg(pose,pose_msg);

    poses.poses.push_back(pose_msg);
  }

  // last pose
  pose_msg = poses.poses.back();
  pose_msg.position.x = cloud_normals[indices.back()].x;
  pose_msg.position.y = cloud_normals[indices.back()].y;
  pose_msg.position.z = cloud_normals[indices.back()].z;
  poses.poses.push_back(pose_msg);

  return true;
}

namespace tool_path_planner
{
EdgePathGenerator::EdgePathGenerator()
{
  // TODO Auto-generated constructor stub
}

EdgePathGenerator::~EdgePathGenerator()
{
  // TODO Auto-generated destructor stub
}

boost::optional< std::vector<geometry_msgs::PoseArray >> EdgePathGenerator::generate(const tool_path_planner::EdgePathConfig& config)
{
  using namespace pcl;

  edge_results_.reset (new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::copyPointCloud(*input_cloud_, *edge_results_);

  // identify high v values
  const int num_threads = config.num_threads;
  long cnt = 0;
  long percentage = 0;
  long num_points = static_cast<long>(input_cloud_->size());
#pragma omp parallel for num_threads(num_threads)
  for (std::size_t idx = 0; idx < input_cloud_->points.size(); ++idx)
  {
    std::vector<int> point_idx;
    std::vector<float> point_squared_distance;
    octree_search_->radiusSearch (static_cast<int>(idx), config.search_radius, point_idx, point_squared_distance);

    Eigen::Vector4d centroid;
    Eigen::Matrix3d covariance_matrix;

    pcl::computeMeanAndCovarianceMatrix (*input_cloud_,
                                         point_idx,
                                         covariance_matrix,
                                         centroid);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.compute(covariance_matrix, false);
    auto eigen_values = solver.eigenvalues();
    double sigma1 = eigen_values(0)/(eigen_values(0) + eigen_values(1) + eigen_values(2));
    double sigma2 = eigen_values(1)/(eigen_values(0) + eigen_values(1) + eigen_values(2));
    double sigma3 = eigen_values(2)/(eigen_values(0) + eigen_values(1) + eigen_values(2));
    edge_results_->points[idx].h = static_cast<float>(sigma1);
    edge_results_->points[idx].s = static_cast<float>(sigma2);
    edge_results_->points[idx].v = static_cast<float>(sigma3); // will use this value for edge detection

#pragma omp critical
    {
      ++cnt;
      if (static_cast<long>(100.0f * static_cast<float>(cnt)/num_points) > percentage)
      {
        percentage = static_cast<long>(100.0f * static_cast<float>(cnt)/num_points);
        std::stringstream ss;
        ss << "\rPoints Processed: " << percentage << "% of " << num_points;
        CONSOLE_BRIDGE_logInform("%s", ss.str().c_str());
      }
    }
  }

  pcl::PointIndices edge_indices;
  if(!filterEdges(edge_results_, edge_indices, config.neighbor_tol))
  {
    CONSOLE_BRIDGE_logError("Failed to filter edge points");
    return boost::none;
  }

  CONSOLE_BRIDGE_logInform("Found %lu edge points", edge_indices.indices.size());

  // cluster into individual segments
  std::vector<PointIndices> cluster_indices;
  PointCloud<PointXYZ>::Ptr cloud_points = boost::make_shared<PointCloud<PointXYZ>>();
  pcl::copyPointCloud(*input_cloud_,*cloud_points);
  if(config.cluster_max > 0 && config.cluster_max > config.cluster_min)
  {
    pcl::PointIndices::Ptr remaining_edge_indices = boost::make_shared<pcl::PointIndices>(edge_indices);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(cloud_points);
    ec.setIndices(remaining_edge_indices);
    ec.setClusterTolerance(config.cluster_tol);
    ec.setMinClusterSize(config.cluster_min);
    ec.setMaxClusterSize(config.cluster_max);
    ec.extract(cluster_indices);
    CONSOLE_BRIDGE_logInform("Found %lu edge clusters",cluster_indices.size());
  }
/*  else
  {
    CONSOLE_BRIDGE_logInform("Skipping clustering");
    cluster_indices.push_back(edge_indices);
  }*/

  // split into segments using octree
  splitEdgeSegments(config,cloud_points,edge_indices,cluster_indices);

  // generating paths from the segments
  std::vector<geometry_msgs::PoseArray> edge_paths;
  std::size_t cluster_idx = 0;;
  for(PointIndices& idx : cluster_indices)
  {
    // rearrange so that the points follow a consistent order
    PointIndices rearranged_cluster;
    CONSOLE_BRIDGE_logInform("Reordering edge cluster %lu", cluster_idx);
    if(!reorder(config, cloud_points,idx, rearranged_cluster.indices))
    {
      return boost::none;
    }

    // converting the rearranged point segments into poses
    geometry_msgs::PoseArray path_poses;
    CONSOLE_BRIDGE_logInform("Converting edge cluster %lu to poses", cluster_idx);
    if(!createPoseArray(*input_cloud_,rearranged_cluster.indices, path_poses))
    {
      return boost::none;
    }
    edge_paths.push_back(path_poses);
    cluster_idx++;
  }
  return edge_paths;
}

void EdgePathGenerator::setInput(pcl::PolygonMesh::ConstPtr mesh,  double octree_res)
{
  using namespace pcl;
  mesh_ = mesh;
  input_cloud_ = boost::make_shared<PointCloud<PointNormal>>();
  noether_conversions::convertToPointNormals(*mesh, *input_cloud_);
  PointCloud<PointXYZ>::Ptr input_points = boost::make_shared< PointCloud<PointXYZ> >();
  pcl::copyPointCloud(*input_cloud_, *input_points );
  octree_search_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(octree_res));
  octree_search_->setInputCloud (input_points);
  octree_search_->addPointsFromInputCloud ();
}

void EdgePathGenerator::setInput(const shape_msgs::Mesh& mesh, double octree_res)
{
  pcl::PolygonMesh::Ptr pcl_mesh = boost::make_shared<pcl::PolygonMesh>();
  noether_conversions::convertToPCLMesh(mesh,*pcl_mesh);
  setInput(pcl_mesh, octree_res);
}

boost::optional<std::vector<geometry_msgs::PoseArray>>
EdgePathGenerator::generate(const shape_msgs::Mesh& mesh, const tool_path_planner::EdgePathConfig& config)
{
  setInput(mesh,config.octree_res);
  return generate(config);
}

boost::optional<std::vector<geometry_msgs::PoseArray>>
EdgePathGenerator::generate(pcl::PolygonMesh::ConstPtr mesh, const tool_path_planner::EdgePathConfig& config)
{
  setInput(mesh,config.octree_res);
  return generate(config);
}

bool EdgePathGenerator::splitEdgeSegments(const tool_path_planner::EdgePathConfig& config,
                                          pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
                                          const pcl::PointIndices& edge_indices,
                                          std::vector<pcl::PointIndices>& segments)
{
  using namespace Eigen;
  using PType = std::remove_reference<decltype(*points)>::type::PointType;
  pcl::PointIndices current_segment_indices= edge_indices;
  std::vector< int > k_indices;
  bool split = false;
  int split_idx = -1;
  for(std::size_t i =0 ; i < current_segment_indices.indices.size() -1; i++)
  {
    const PType& p1 = (*points)[current_segment_indices.indices[i]];
    const PType& p2 = (*points)[current_segment_indices.indices[i+1]];
    Vector3f dir = (p2.getVector3fMap() - p1.getVector3fMap());
    k_indices.clear();
    int voxels_encountered = octree_search_->getIntersectedVoxelIndices(p1.getVector3fMap(),
                                                                        dir,k_indices, config.max_intersecting_voxels + 1);
    if(voxels_encountered > config.max_intersecting_voxels)
    {
      // splitting current segment into two
      split = true;
      split_idx = i;
      CONSOLE_BRIDGE_logInform("Splitting at index %i, encountered %i voxels", split_idx, voxels_encountered);
      break;
    }
  }

  if(split)
  {
    pcl::PointIndices next_segment_indices;
    auto next_segment_start_iter = std::next(current_segment_indices.indices.begin(),split_idx + 1);
    next_segment_indices.indices.assign(next_segment_start_iter,current_segment_indices.indices.end());
    current_segment_indices.indices.erase(next_segment_start_iter,current_segment_indices.indices.end());
    segments.push_back(current_segment_indices);

    if(next_segment_indices.indices.size() < 2)
    {
      CONSOLE_BRIDGE_logWarn("Only one point was found in the next segment, will ignore");
      return true;
    }

    return splitEdgeSegments(config, points,next_segment_indices, segments);
  }

  segments.push_back(current_segment_indices);
  return true;
}
} /* namespace tool_path_planner */

