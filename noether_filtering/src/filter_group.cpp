#include "noether_filtering/filter_group.hpp"
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#define PCL_INSTANTIATE_FilterGroup(T) \
  template class PCL_EXPORTS noether_filtering::FilterGroup<pcl::PointCloud<T>::Ptr>;

namespace noether_filtering
{
// Explicit template instantiation
template<>
class FilterGroup<pcl::PolygonMesh>;

// Explicit template instantiation for all XYZ point types
PCL_INSTANTIATE(FilterGroup, PCL_XYZ_POINT_TYPES);

} // namespace noether_filtering
