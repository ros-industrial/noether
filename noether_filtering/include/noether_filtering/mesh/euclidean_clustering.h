/*
 * euclidean_clustering.h
 *
 *  Created on: Nov 26, 2019
 *      Author: jrgnicho
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
class EuclideanClustering: public MeshFilterBase
{
public:

  struct Parameters
  {
    double tolerance = 0.02; // meters
    int min_cluster_size = 100;
    int max_cluster_size = -1; // will use input point cloud size when negative
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
