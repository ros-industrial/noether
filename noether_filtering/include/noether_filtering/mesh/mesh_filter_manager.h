#ifndef NOETHER_FILTERING_MESH_MESH_FILTER_H
#define NOETHER_FILTERING_MESH_MESH_FILTER_H

#include "noether_filtering/filter_manager.h"

// Forward declare PCL types
namespace pcl
{
class PolygonMesh;
}

namespace noether_filtering
{

template<typename T>
class FilterGroup;

template<typename T>
class FilterManager;

// Provide aliases for specific types
using MeshFilterGroup = FilterGroup<pcl::PolygonMesh>;
using MeshFilterManager = FilterManager<pcl::PolygonMesh>;

}

#endif // NOETHER_FILTERING_MESH_MESH_FILTER_H
