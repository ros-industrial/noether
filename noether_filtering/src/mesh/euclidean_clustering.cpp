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

#include "noether_filtering/mesh/euclidean_clustering.h"
#include "noether_filtering/utils.h"
#include <boost/make_shared.hpp>
#include <console_bridge/console.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <XmlRpcException.h>

namespace noether_filtering
{
namespace mesh
{
bool EuclideanClustering::configure(XmlRpc::XmlRpcValue config)
{
  std::vector<std::string> fields = { "tolerance", "min_cluster_size", "max_cluster_size" };
  if (!std::all_of(fields.begin(), fields.end(), [this, &config](const decltype(fields)::value_type& s) {
        if (!config.hasMember(s))
        {
          CONSOLE_BRIDGE_logError("%s did not find the %s field in the configuration", getName().c_str(), s.c_str());
          return false;
        }
        return true;
      }))
  {
    return false;
  }

  try
  {
    int idx = 0;
    parameters_.tolerance = static_cast<double>(config[fields[idx++]]);
    parameters_.min_cluster_size = static_cast<int>(config[fields[idx++]]);
    parameters_.max_cluster_size = static_cast<int>(config[fields[idx++]]);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    CONSOLE_BRIDGE_logError("%s Failed to cast a configuration field, %s", getName().c_str(), e.getMessage().c_str());
    return false;
  }

  return true;
}

bool EuclideanClustering::filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out)
{
  using namespace pcl;
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  // converting to point cloud
  Cloud::Ptr mesh_points = boost::make_shared<Cloud>();
  pcl::fromPCLPointCloud2(mesh_in.cloud, *mesh_points);

  // computing clusters
  search::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(mesh_points);
  std::vector<pcl::PointIndices> cluster_indices;
  EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(parameters_.tolerance);
  ec.setMinClusterSize(parameters_.min_cluster_size);
  ec.setMaxClusterSize(parameters_.max_cluster_size <= 0 ? mesh_points->size() : parameters_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(mesh_points);
  ec.extract(cluster_indices);

  if (cluster_indices.empty())
  {
    CONSOLE_BRIDGE_logError("%s found no clusters", getName().c_str());
    return false;
  }

  CONSOLE_BRIDGE_logInform("%s found %lu clusters", getName().c_str(), cluster_indices.size());
  std::for_each(
      cluster_indices.begin(), cluster_indices.end(), [idx = 0](decltype(cluster_indices)::value_type& c) mutable {
        CONSOLE_BRIDGE_logInform("\t cluster[%i] -> %lu points ", idx++, c.indices.size());
      });

  // accumulate all cluster indices into one
  std::vector<int> combined_indices;
  std::for_each(
      cluster_indices.begin(), cluster_indices.end(), [&combined_indices](decltype(cluster_indices)::value_type& c) {
        combined_indices.insert(combined_indices.end(), c.indices.begin(), c.indices.end());
      });

  // sorting
  std::sort(combined_indices.begin(), combined_indices.end(), std::less<int>());
  decltype(combined_indices)::iterator last = std::unique(combined_indices.begin(), combined_indices.end());
  combined_indices.erase(last, combined_indices.end());
  CONSOLE_BRIDGE_logInform("%s clusters contains %lu unique points from the original %lu",
                           getName().c_str(),
                           combined_indices.size(),
                           mesh_points->size());

  // obtaining ignored indices
  std::vector<int> removed_indices;
  removed_indices.reserve(mesh_points->size() - combined_indices.size());
  for (std::size_t i = 1; i < combined_indices.size(); i++)
  {
    const int& v1 = combined_indices[i - 1];
    const int& v2 = combined_indices[i];
    int diff = v2 - v1;
    if (std::abs(diff) > 1)
    {
      std::vector<int> vals(diff - 1);
      std::iota(vals.begin(), vals.end(), v1 + 1);
      removed_indices.insert(removed_indices.end(), vals.begin(), vals.end());
    }
  }

  auto is_valid_polygon = [& points = *mesh_points](const Vertices& polygon, double tol = 1e-6) -> bool {
    Eigen::Vector3f a, b, c, dir;
    const decltype(polygon.vertices)& vert = polygon.vertices;
    a = points[vert[0]].getVector3fMap();
    b = points[vert[1]].getVector3fMap();
    c = points[vert[2]].getVector3fMap();

    dir = ((b - a).cross((c - a))).normalized();
    float norm = dir.norm();
    return !(std::isnan(norm) || std::isinf(norm));
  };

  // iterating over the polygons and keeping those in the clusters
  decltype(mesh_in.polygons) remaining_polygons;
  for (const auto& plgn : mesh_in.polygons)
  {
    if (!is_valid_polygon(plgn))
    {
      continue;
    }

    decltype(removed_indices)::iterator pos;
    bool add_poly = true;
    for (const auto& v : plgn.vertices)
    {
      pos = std::find(removed_indices.begin(), removed_indices.end(), v);
      if (pos != removed_indices.end())
      {
        // add the polygon and exit
        add_poly = false;
        break;
      }
    }

    if (add_poly)
    {
      remaining_polygons.push_back(plgn);
    }
  }

  if (remaining_polygons.empty())
  {
    CONSOLE_BRIDGE_logError("%s found no remaining polygons", getName().c_str());
    return false;
  }

  CONSOLE_BRIDGE_logInform("New mesh contains %lu polygons from %lu in the original one",
                           remaining_polygons.size(),
                           mesh_in.polygons.size());

  // creating new polygon mesh
  pcl::PolygonMesh reduced_mesh;
  reduced_mesh = mesh_in;
  reduced_mesh.polygons = remaining_polygons;
  surface::SimplificationRemoveUnusedVertices mesh_simplification;
  mesh_simplification.simplify(reduced_mesh, mesh_out);
  return true;
}

std::string EuclideanClustering::getName() const { return utils::getClassName<decltype(*this)>(); }

} /* namespace mesh */
} /* namespace noether_filtering */
