#include "noether_filtering/cloud/impl/voxel_grid_filter_impl.hpp"
#include "noether_filtering/cloud/impl/statistical_outlier_filter.hpp"
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

namespace noether_filtering
{
// Explicit template instantiation for all XYZ point types
PCL_INSTANTIATE(VoxelGridFilter, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(StatisticalOutlierFilter, PCL_XYZ_POINT_TYPES);

} // namespace noether_filtering
