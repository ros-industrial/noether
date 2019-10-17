#ifndef NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_HPP

#include <console_bridge/console.h>
#include "noether_filtering/cloud/voxel_grid_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/filters/voxel_grid.h>
#include <XmlRpcException.h>

namespace noether_filtering
{
template<typename PointT>
bool VoxelGridFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  // Required parameter(s)
  try
  {
    params.leaf_size = static_cast<double>(value["leaf_size"]);
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    CONSOLE_BRIDGE_logError("Failed to load required parameter(s) for voxel grid filter: '%s'",
                            ex.getMessage().c_str());
    return false;
  }

  // Optional parameter(s)
  try
  {
    params.filter_field_name = static_cast<std::string>(value["filter_field_name"]);
    params.min_limit = static_cast<double>(value["min_limit"]);
    params.max_limit = static_cast<double>(value["max_limit"]);
    params.filter_limits_negative = static_cast<bool>(value["filter_limits_negative"]);
    params.min_pts_per_voxel = static_cast<int>(value["min_pts_per_voxel"]);
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load optional parameter(s) for voxel grid filter: '%s'",
                           ex.getMessage().c_str());
  }

  return true;
}

template<typename PointT>
bool VoxelGridFilter<PointT>::filter(const T &input, T &output)
{
  output.reset(new pcl::PointCloud<PointT>());

  pcl::VoxelGrid<PointT> f;

  // Set the parameters
  f.setLeafSize(params.leaf_size, params.leaf_size, params.leaf_size);
  f.setMinimumPointsNumberPerVoxel(params.min_pts_per_voxel);

  if(!params.filter_field_name.empty())
  {
    f.setFilterFieldName(params.filter_field_name);
    f.setFilterLimits(params.min_limit, params.max_limit);
    f.setFilterLimitsNegative(params.filter_limits_negative);
  }

  f.setInputCloud(input);
  f.filter(*output);

  return true;
}

template<typename PointT>
std::string VoxelGridFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(this)>();
}

} // namespace noether_filtering

#define PCL_INSTANTIATE_VoxelGridFilter(T) \
  template class PCL_EXPORTS noether_filtering::VoxelGridFilter<T>;

#endif // NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_HPP
