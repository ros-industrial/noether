#ifndef NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_IMPL_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_IMPL_HPP

#include "noether_filtering/cloud/voxel_grid_filter.h"
#include <XmlRpcException.h>
#include <console_bridge/console.h>

namespace noether_filtering
{
template<typename PointT>
bool VoxelGridFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  // Required parameter(s)
  try
  {
    params_.leaf_size = static_cast<double>(value["leaf_size"]);
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
    params_.filter_field_name = static_cast<std::string>(value["filter_field_name"]);
    params_.min_limit = static_cast<double>(value["min_limit"]);
    params_.max_limit = static_cast<double>(value["max_limit"]);
    params_.filter_limits_negative = static_cast<bool>(value["filter_limits_negative"]);
    params_.min_pts_per_voxel = static_cast<int>(value["min_pts_per_voxel"]);
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

  // Set the parameters
  filter_.setLeafSize(params_.leaf_size, params_.leaf_size, params_.leaf_size);
  filter_.setMinimumPointsNumberPerVoxel(params_.min_pts_per_voxel);

  if(!params_.filter_field_name.empty())
  {
    filter_.setFilterFieldName(params_.filter_field_name);
    filter_.setFilterLimits(params_.min_limit, params_.max_limit);
    filter_.setFilterLimitsNegative(params_.filter_limits_negative);
  }

  filter_.setInputCloud(input);
  filter_.filter(*output);

  return true;
}

} // namespace noether_filtering

#define PCL_INSTANTIATE_VoxelGridFilter(T) \
  template class PCL_EXPORTS noether_filtering::VoxelGridFilter<T>;

#endif // NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_IMPL_HPP
