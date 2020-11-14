#ifndef NOETHER_FILTERING_MESH_MESH_FILTER_BASE_H
#define NOETHER_FILTERING_MESH_MESH_FILTER_BASE_H

#include <pcl/PolygonMesh.h>
#include "noether_filtering/filter_base.h"

namespace noether_filtering
{
namespace mesh
{
// Provide alias for mesh filter base
using MeshFilterBase = FilterBase<pcl::PolygonMesh>;

} /* namespace mesh */
} /* namespace noether_filtering */

#endif  // NOETHER_FILTERING_MESH_MESH_FILTER_BASE_H
