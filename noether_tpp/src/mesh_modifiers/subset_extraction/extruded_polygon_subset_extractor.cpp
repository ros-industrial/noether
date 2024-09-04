/*
 * Copyright 2018 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <noether_tpp/mesh_modifiers/subset_extraction/extruded_polygon_subset_extractor.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/pcl_config.h>

namespace
{
#if PCL_VERSION_COMPARE(<, 1, 10, 0)
template <typename T>
using shared_ptr = std::shared_ptr<T>;

template <typename T>
auto make_shared = std::make_shared<T>;
#else
template <typename T>
using shared_ptr = pcl::shared_ptr<T>;

template <typename T>
auto make_shared = pcl::make_shared<T>;
#endif

/** @brief Creates a shared pointer with no-op destructor for various PCL methods that require a pointer */
template <typename T>
shared_ptr<const T> createLocalPointer(const T& obj)
{
  return shared_ptr<const T>(&obj, [](const T*) {});
}

bool clusterComparator(const pcl::PointIndices& a,
                       const pcl::PointIndices& b,
                       pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                       const Eigen::Vector3f& boundary_centroid)
{
  // Add cluster indices to vector as shared pointers with empty destructors
  std::vector<pcl::PointIndices::ConstPtr> cluster_indices{ createLocalPointer(a), createLocalPointer(b) };

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);

  // Find the distance between the selection polygon origin and the centroid of each cluster
  std::vector<float> dist;
  for (auto indices : cluster_indices)
  {
    extract.setIndices(indices);
    pcl::PointCloud<pcl::PointXYZ> index_cloud;
    extract.filter(index_cloud);

    float avg_dist = (index_cloud.getMatrixXfMap(3, 4, 0).colwise() - boundary_centroid).rowwise().mean().norm();

    dist.push_back(avg_dist);
  }

  return dist.front() < dist.back();
}

Eigen::Hyperplane<double, 3> fitPlaneToPoints(const Eigen::MatrixX3d& points, const double plane_distance_threshold)
{
  // Create a point cloud from the selection points
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  input_cloud->resize(points.rows());
  input_cloud->getMatrixXfMap(3, 4, 0) = points.cast<float>().transpose();

  // Set segmentation parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(plane_distance_threshold);
  seg.setInputCloud(input_cloud);

  // Extract best fit cylinder inliers and calculate cylinder coefficients
  auto plane_coefficients = make_shared<pcl::ModelCoefficients>();
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
  seg.segment(*plane_inliers, *plane_coefficients);

  if (plane_inliers->indices.size() == 0)
    throw std::runtime_error("Unable to fit a plane to the data");

  Eigen::Hyperplane<float, 3> plane;
  plane.coeffs() = Eigen::Map<Eigen::Vector4f>(plane_coefficients->values.data());

  return plane.cast<double>();
}

std::vector<int> getSubMeshIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr vertices,
                                   const Eigen::MatrixX3d& projected_boundary,
                                   const noether::ExtrudedPolygonSubsetExtractor::Params& params)
{
  // Calculate the diagonal of search_points_ cloud
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*vertices, min_pt, max_pt);
  float half_dist = (max_pt.getVector3fMap() - min_pt.getVector3fMap()).norm() / 2.0;

  // Create a point cloud from the projected boundary
  auto projected_boundary_cloud = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  projected_boundary_cloud->resize(projected_boundary.rows());
  projected_boundary_cloud->getMatrixXfMap(3, 4, 0) = projected_boundary.cast<float>().transpose();

  // Extrude the convex hull by half the max distance
  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
  pcl::PointIndicesPtr selection_indices(new pcl::PointIndices{});
  prism.setInputCloud(vertices);
  prism.setInputPlanarHull(projected_boundary_cloud);
  prism.setHeightLimits(-half_dist, half_dist);
  prism.segment(*selection_indices);

  if (selection_indices->indices.empty())
    throw std::runtime_error("No points found within polygon boundary");

  // Pull out the points that are inside the user selected prism
  auto prism_indices = make_shared<pcl::PointIndices>();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(vertices);
  extract.setIndices(selection_indices);
  extract.setNegative(false);
  extract.filter(prism_indices->indices);

  // Extract the clusters within the boundary
  std::vector<pcl::PointIndices> cluster_indices;
  {
    auto tree = make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
    cluster.setInputCloud(vertices);
    cluster.setIndices(prism_indices);
    cluster.setMinClusterSize(params.min_cluster_size);
    cluster.setMaxClusterSize(params.max_cluster_size);
    cluster.setClusterTolerance(params.cluster_tolerance);

    cluster.extract(cluster_indices);
  }

  // Find the cluster closest to the centroid of the selection polygon
  Eigen::Vector3d boundary_centroid = projected_boundary.colwise().mean();

  // Find the closest cluster
  auto closest_cluster = std::min_element(
      cluster_indices.begin(),
      cluster_indices.end(),
      std::bind(
          &clusterComparator, std::placeholders::_1, std::placeholders::_2, vertices, boundary_centroid.cast<float>()));

  if (closest_cluster == cluster_indices.end())
    throw std::runtime_error("No clusters found");

  return closest_cluster->indices;
}

}  // namespace

namespace noether
{
std::vector<int> ExtrudedPolygonSubsetExtractor::extract(const pcl::PCLPointCloud2& cloud,
                                                         const Eigen::MatrixX3d& boundary) const
{
  // Check size of selection points vector
  if (boundary.rows() < 3)
    throw std::runtime_error("At least three points required to create a closed loop");

  // Convert to point cloud class
  auto vertices = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(cloud, *vertices);

  // Check the size of the input point cloud
  if (vertices->points.size() == 0)
    throw std::runtime_error("No points to search for inside the selection polygon");

  // Take boundary points and fit a plane to them using RANSAC
  Eigen::Hyperplane<double, 3> plane = fitPlaneToPoints(boundary, params.plane_distance_threshold);

  // Project the selection points onto the fitted plane
  // Note: Eigen does not appear to support a vectorized form of the `projection` method so a for loop is required
  Eigen::MatrixX3d projected_boundary(boundary.rows(), 3);
  for (Eigen::Index i = 0; i < boundary.rows(); ++i)
  {
    projected_boundary.row(i) = plane.projection(boundary.row(i));
  }

  /* Find all of the sensor data points inside a volume whose perimeter is defined by the projected selection points
   * and whose depth is defined by the largest length of the bounding box of the input point cloud */
  std::vector<int> subset_vertex_indices = getSubMeshIndices(vertices, projected_boundary, params);
  if (subset_vertex_indices.empty())
    throw std::runtime_error("Unable to identify points in the region of interest");

  return subset_vertex_indices;
}

pcl::PolygonMesh ExtrudedPolygonSubMeshExtractor::extract(const pcl::PolygonMesh& mesh,
                                                          const Eigen::MatrixX3d& boundary) const
{
  std::vector<int> subset_indices = extractor.extract(mesh.cloud, boundary);
  return extractSubMeshFromInlierVertices(mesh, subset_indices);
}

}  // namespace noether
