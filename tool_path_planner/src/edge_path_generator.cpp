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
  range_cond->addComparison(boost::make_shared< FieldComparison<PType> >("v", ComparisonOps::GT,max));

  // build the filter
  ConditionalRemoval<PType> cond_filter;
  cond_filter.setCondition(range_cond);
  cond_filter.setInputCloud(input);
  cond_filter.setKeepOrganized(true);

  pcl::PointCloud<PType> filtered_cloud;
  cond_filter.filter(filtered_cloud);
  cond_filter.getRemovedIndices(indices);
  return indices.indices.empty();
}

bool reorder(const tool_path_planner::EdgePathConfig& config,
             pcl::PointCloud<pcl::PointXYZ>::ConstPtr points ,const pcl::PointIndices cluster_indices,
             std::vector<int>& rearranged_indices)
{
  using namespace pcl;
  pcl::IndicesPtr active_ind_ptr = boost::make_shared<decltype(cluster_indices.indices)>(cluster_indices.indices);
  std::vector<int>& active_indices = *active_ind_ptr;

  // downsampling first
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = boost::make_shared<PointCloud<PointXYZ>>();
  pcl::VoxelGrid<PointXYZ> voxelgrid;
  voxelgrid.setInputCloud(points);
  voxelgrid.setIndices(active_ind_ptr);
  voxelgrid.setLeafSize(config.voxel_size, config.voxel_size, config.voxel_size);
  voxelgrid.filter(*cloud_filtered);
  std::vector<int> removed_ind = *voxelgrid.getRemovedIndices();
  active_indices.erase(std::remove_if(active_indices.begin(), active_indices.end(),[&](const int& v){
    return std::find(removed_ind.begin(), removed_ind.end(),v) != removed_ind.end();
  }));


  // build kd tree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setEpsilon(config.kdtree_epsilon);


  auto& cloud = *points;
  std::vector<int> visited_indices;
  visited_indices.reserve(active_indices.size());
  int search_point_idx = active_indices[0];
  visited_indices.push_back(search_point_idx); // insert first point

  const int k_points = 1;
  const int max_iters = active_indices.size();
  PointXYZ search_point = cloud[search_point_idx];
  PointXYZ start_point = search_point;
  PointXYZ next_point;
  int iter_count = 0;

  while(iter_count <= max_iters)
  {
    iter_count++;

    // remove from active
    active_indices.erase(std::remove(active_indices.begin(), active_indices.end(),search_point_idx));

    if(active_indices.empty())
    {
      break;
    }

    // set tree inputs;
    kdtree.setInputCloud(points,active_ind_ptr);

    // find next point
    std::vector<int> k_indices(k_points);
    std::vector<float> k_sqr_distances(k_points);
    int points_found = kdtree.nearestKSearch(search_point,k_points, k_indices, k_sqr_distances);
    if(points_found == 0)
    {
      break;
    }

    // insert new point if it has not been visited
    if(std::find(visited_indices.begin(), visited_indices.end(),k_indices[0]) == visited_indices.end())
    {
      // there should be no more points to add
      break;
    }

    // check if found point is further away than the start point
    next_point = cloud[k_indices[0]];
    start_point = cloud[visited_indices[0]];
    Eigen::Vector3d diff = (start_point.getArray3fMap() - next_point.getArray3fMap()).cast<double>();
    if(diff.norm() < k_sqr_distances[0])
    {
      // start adding points at the other end
      std::reverse(visited_indices.begin(),visited_indices.end());
    }

    // add to visited
    visited_indices.push_back(k_indices[0]);

    // set next search point
    search_point_idx = k_indices[0];
    search_point = next_point;
  }

  if(visited_indices.size() < max_iters)
  {
    CONSOLE_BRIDGE_logError("Failed to include all points in the downsampled group");
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

  auto point_to_vec = [](const PointNormal& p)
  {
    return Vector3d(p.x, p.y, p.z);
  };

  auto to_rotation_mat = [](const Vector3d& vx, const Vector3d& vy, const Vector3d& vz)
  {
    Matrix3d rot;
    rot.row(0) = Vector3d(vx.x(), vy.x(), vz.x());
    rot.row(1) = Vector3d(vx.y(), vy.y(), vz.y());
    rot.row(2) = Vector3d(vx.z(), vy.z(), vz.z());
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

    pose = Translation3d(point_to_vec(p1))* AngleAxisd(to_rotation_mat(x_dir, y_dir, z_dir));
    tf::poseEigenToMsg(pose,pose_msg);
    poses.poses.push_back(pose_msg);
  }

  // last pose
  pose_msg = poses.poses.back();
  pose_msg.position.x = cloud_normals.back().x;
  pose_msg.position.y = cloud_normals.back().y;
  pose_msg.position.z = cloud_normals.back().z;
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
  pcl::PointIndices::Ptr remaining_edge_indices = boost::make_shared<pcl::PointIndices>(edge_indices);
  PointCloud<PointXYZ>::Ptr cloud_points = boost::make_shared<PointCloud<PointXYZ>>();
  pcl::copyPointCloud(*input_cloud_,*cloud_points);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud(cloud_points);
  ec.setIndices(remaining_edge_indices);
  ec.setClusterTolerance(config.cluster_tol);
  ec.setMinClusterSize(config.cluster_min);
  ec.setMaxClusterSize(config.cluster_max);
  std::vector<PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  // generating paths from the segments
  std::vector<geometry_msgs::PoseArray> edge_paths;
  for(PointIndices& idx : cluster_indices)
  {
    // rearrange so that the points follow a consistent order
    PointIndices rearranged_cluster;
    if(!reorder(config, cloud_points,idx, rearranged_cluster.indices))
    {
      return boost::none;
    }

    // converting the rearranged point segments into poses
    geometry_msgs::PoseArray path_poses;
    if(!createPoseArray(*input_cloud_,rearranged_cluster.indices, path_poses))
    {
      return boost::none;
    }
    edge_paths.push_back(path_poses);
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

} /* namespace tool_path_planner */
