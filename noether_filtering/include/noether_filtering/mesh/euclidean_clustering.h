/**
 * @author Jorge Nicho
 * @file euclidean_clustering.h
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

#ifndef INCLUDE_NOETHER_FILTERING_MESH_EUCLIDEAN_CLUSTERING_H_
#define INCLUDE_NOETHER_FILTERING_MESH_EUCLIDEAN_CLUSTERING_H_

#include <limits>
#include <string>
#include "noether_filtering/mesh/mesh_filter_base.h"

namespace noether_filtering
{
namespace mesh
{
class EuclideanClustering : public MeshFilterBase
{
public:
  struct Parameters
  {
    double tolerance = 0.02;  // meters
    int min_cluster_size = 100;
    int max_cluster_size = -1;  // will use input point cloud size when negative
  };

  bool configure(XmlRpc::XmlRpcValue config) override final;
  bool filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out) override final;
  std::string getName() const override final;

protected:
  Parameters parameters_;
};

} /* namespace mesh */
} /* namespace noether_filtering */

#endif /* INCLUDE_NOETHER_FILTERING_MESH_EUCLIDEAN_CLUSTERING_H_ */
