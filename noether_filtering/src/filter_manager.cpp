#include "noether_filtering/filter_manager.hpp"
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#define PCL_INSTANTIATE_FilterManager(T) \
  template class PCL_EXPORTS noether_filtering::FilterManager<pcl::PointCloud<T>::Ptr>;

namespace noether_filtering
{
// Explicit template instantiation
template class FilterManager<pcl::PolygonMesh>;

// Explicit template instantiation for all XYZ point types
PCL_INSTANTIATE(FilterManager, PCL_XYZ_POINT_TYPES);

} // namespace noether_filtering
