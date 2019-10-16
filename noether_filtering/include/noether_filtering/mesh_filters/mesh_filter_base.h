#ifndef INCLUDE_NOETHER_FILTERING_MESH_FILTERS_BASE_H_
#define INCLUDE_NOETHER_FILTERING_MESH_FILTERS_BASE_H_

#include <pcl/PolygonMesh.h>
#include "noether_filtering/filter_base.h"
namespace noether_filtering
{
namespace mesh_filters
{
  typedef FilterBase<pcl::PolygonMesh> MeshFilterBase;
}
}

#endif
