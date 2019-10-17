#include "noether_filtering/cloud/impl/voxel_grid_filter.hpp"
#include "noether_filtering/cloud/impl/statistical_outlier_filter.hpp"
#include "noether_filtering/cloud/impl/crop_box_filter.hpp"
#include "noether_filtering/cloud/impl/pass_through_filter.hpp"
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

namespace noether_filtering
{
// Explicit template instantiation for all XYZ point types
PCL_INSTANTIATE(VoxelGridFilter, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(StatisticalOutlierFilter, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(CropBoxFilter, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(PassThroughFilter, PCL_XYZ_POINT_TYPES);

} // namespace noether_filtering
