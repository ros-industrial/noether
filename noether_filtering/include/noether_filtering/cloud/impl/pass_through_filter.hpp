#ifndef NOETHER_FILTERING_CLOUD_IMPL_PASS_THROUGH_FILTER_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_PASS_THROUGH_FILTER_HPP

#include <console_bridge/console.h>
#include "noether_filtering/cloud/pass_through_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/filters/passthrough.h>
#include <XmlRpcException.h>

namespace noether_filtering
{
template<typename PointT>
bool PassThroughFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  // Required parameter(s)
  try
  {
    params.filter_field_name = static_cast<std::string>(value["filter_field_name"]);
    params.min_limit = static_cast<double>(value["min_limit"]);
    params.max_limit = static_cast<double>(value["max_limit"]);
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    CONSOLE_BRIDGE_logError("Failed to load required parameter(s) for pass through filter: '%s'",
                            ex.getMessage().c_str());
    return false;
  }

  // Optional parameter(s)
  try
  {
    params.negative = static_cast<bool>(value["negative"]);
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    CONSOLE_BRIDGE_logWarn("Failed to load optional parameter(s) for pass through filter: '%s'",
                           ex.getMessage().c_str());
  }

  return true;
}

template<typename PointT>
bool PassThroughFilter<PointT>::filter(const T &input, T &output)
{
  output.reset(new pcl::PointCloud<PointT>());

  pcl::PassThrough<PointT> f;

  // Set the parameters
  if(!params.filter_field_name.empty())
  {
    f.setFilterFieldName(params.filter_field_name);
    f.setFilterLimits(params.min_limit, params.max_limit);
    f.setFilterLimitsNegative(params.negative);
  }
  else
  {
    CONSOLE_BRIDGE_logError("No filter field name set for pass through filter");
    return false;
  }

  f.setInputCloud(input);
  f.filter(*output);

  return true;
}

template<typename PointT>
std::string PassThroughFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(this)>();
}

} // namespace noether_filtering

#define PCL_INSTANTIATE_PassThroughFilter(T) \
  template class PCL_EXPORTS noether_filtering::PassThroughFilter<T>;

#endif // NOETHER_FILTERING_CLOUD_IMPL_PASS_THROUGH_FILTER_HPP
