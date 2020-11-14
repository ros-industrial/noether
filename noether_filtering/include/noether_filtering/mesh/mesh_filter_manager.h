#ifndef NOETHER_FILTERING_MESH_MESH_FILTER_H
#define NOETHER_FILTERING_MESH_MESH_FILTER_H

#include <pcl/PolygonMesh.h>
#include "noether_filtering/filter_manager.h"

namespace noether_filtering
{
namespace mesh
{
// Provide aliases for specific types
using MeshFilterGroup = FilterGroup<pcl::PolygonMesh>;
class MeshFilterManager : public FilterManager<pcl::PolygonMesh>
{
public:
  MeshFilterManager() : FilterManager<pcl::PolygonMesh>("noether_filtering::mesh::MeshFilterBase") {}

  virtual ~MeshFilterManager() = default;
};

} /* namespace mesh */
} /* namespace noether_filtering */

#endif  // NOETHER_FILTERING_MESH_MESH_FILTER_H
