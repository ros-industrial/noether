#ifndef NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_HPP

#include <console_bridge/console.h>
#include "noether_filtering/cloud/voxel_grid_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/filters/voxel_grid.h>
#include <XmlRpcException.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
const std::string VoxelGridFilter<PointT>::LEAF_SIZE = "leaf_size";

template <typename PointT>
const std::string VoxelGridFilter<PointT>::FILTER_FIELD_NAME = "filter_field_name";

template <typename PointT>
const std::string VoxelGridFilter<PointT>::MIN_LIMIT = "min_limit";

template <typename PointT>
const std::string VoxelGridFilter<PointT>::MAX_LIMIT = "max_limit";

template <typename PointT>
const std::string VoxelGridFilter<PointT>::FILTER_LIMITS_NEGATIVE = "filter_limits_negative";

template <typename PointT>
const std::string VoxelGridFilter<PointT>::MIN_PTS_PER_VOXEL = "min_pts_per_voxel";

template <typename PointT>
bool VoxelGridFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  if (!value.hasMember(LEAF_SIZE))
  {
    CONSOLE_BRIDGE_logError("Voxel grid filter missing required configuration parameter: %s", LEAF_SIZE.c_str());
    return false;
  }

  // Required parameter(s)
  try
  {
    params.leaf_size = static_cast<double>(value[LEAF_SIZE]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("Failed to load required parameter(s) for voxel grid filter: '%s'",
                            ex.getMessage().c_str());
    return false;
  }

  // Optional parameter(s)
  if (value.hasMember(FILTER_FIELD_NAME) && value.hasMember(MIN_LIMIT) && value.hasMember(MAX_LIMIT) &&
      value.hasMember(FILTER_LIMITS_NEGATIVE) && value.hasMember(MIN_PTS_PER_VOXEL))
  {
    try
    {
      params.filter_field_name = static_cast<std::string>(value[FILTER_FIELD_NAME]);
      params.min_limit = static_cast<double>(value[MIN_LIMIT]);
      params.max_limit = static_cast<double>(value[MAX_LIMIT]);
      params.filter_limits_negative = static_cast<bool>(value[FILTER_LIMITS_NEGATIVE]);
      params.min_pts_per_voxel = static_cast<int>(value[MIN_PTS_PER_VOXEL]);
    }
    catch (const XmlRpc::XmlRpcException& ex)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load optional parameter(s) for voxel grid filter: '%s'",
                             ex.getMessage().c_str());
    }
  }

  return true;
}

template <typename PointT>
bool VoxelGridFilter<PointT>::filter(const T& input, T& output)
{
  // Create a shared pointer to the input object with a "destructor" function that does not delete the raw pointer
  auto cloud = boost::shared_ptr<const T>(&input, [](const T*) {});

  pcl::VoxelGrid<PointT> f;

  // Set the parameters
  f.setLeafSize(params.leaf_size, params.leaf_size, params.leaf_size);
  f.setMinimumPointsNumberPerVoxel(params.min_pts_per_voxel);

  if (!params.filter_field_name.empty())
  {
    f.setFilterFieldName(params.filter_field_name);
    f.setFilterLimits(params.min_limit, params.max_limit);
    f.setFilterLimitsNegative(params.filter_limits_negative);
  }

  f.setInputCloud(cloud);
  f.filter(output);

  return true;
}

template <typename PointT>
std::string VoxelGridFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(this)>();
}

}  // namespace cloud
}  // namespace noether_filtering

#define PCL_INSTANTIATE_VoxelGridFilter(T) template class PCL_EXPORTS noether_filtering::cloud::VoxelGridFilter<T>;

#endif  // NOETHER_FILTERING_CLOUD_IMPL_VOXEL_GRID_FILTER_HPP
