/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file euclidean_clustering.cpp
 * @date Nov 26, 2019
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
#include <noether_tpp/mesh_modifiers/euclidean_clustering_modifier.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace noether
{
std::vector<pcl::PolygonMesh> EuclideanClustering::modify(const pcl::PolygonMesh& mesh_in) const
{
  // converting to point cloud
  auto mesh_points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh_in.cloud, *mesh_points);

  // computing clusters
  auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(mesh_points);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(parameters_.tolerance);
  ec.setMinClusterSize(parameters_.min_cluster_size);
  ec.setMaxClusterSize(parameters_.max_cluster_size <= 0 ? mesh_points->size() : parameters_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(mesh_points);
  ec.extract(cluster_indices);

  if (cluster_indices.empty())
    throw std::runtime_error("No clusters found");

  std::vector<pcl::PolygonMesh> meshes;
  meshes.reserve(cluster_indices.size());
  for (const pcl::PointIndices& cluster : cluster_indices)
    meshes.push_back(extractSubMeshFromInlierVertices(mesh_in, cluster.indices));

  return meshes;
}

}  // namespace noether
