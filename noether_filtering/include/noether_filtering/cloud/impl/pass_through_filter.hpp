#ifndef NOETHER_FILTERING_CLOUD_IMPL_PASS_THROUGH_FILTER_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_PASS_THROUGH_FILTER_HPP

#include <console_bridge/console.h>
#include "noether_filtering/cloud/pass_through_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/filters/passthrough.h>
#include <XmlRpcException.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
const std::string PassThroughFilter<PointT>::FILTER_FIELD_NAME = "filter_field_name";

template <typename PointT>
const std::string PassThroughFilter<PointT>::MIN_LIMIT = "min_limit";

template <typename PointT>
const std::string PassThroughFilter<PointT>::MAX_LIMIT = "max_limit";

template <typename PointT>
const std::string PassThroughFilter<PointT>::NEGATIVE = "negative";

template <typename PointT>
bool PassThroughFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  std::string error;
  if (!value.hasMember(FILTER_FIELD_NAME))
    error += FILTER_FIELD_NAME + ", ";
  if (!value.hasMember(MIN_LIMIT))
    error += MIN_LIMIT + ", ";
  if (!value.hasMember(MAX_LIMIT))
    error += MAX_LIMIT + ", ";

  if (!error.empty())
  {
    CONSOLE_BRIDGE_logError("Pass through filter configuration missing required parameters: %s", error.c_str());
    return false;
  }

  // Required parameter(s)
  try
  {
    params.filter_field_name = static_cast<std::string>(value[FILTER_FIELD_NAME]);
    params.min_limit = static_cast<double>(value[MIN_LIMIT]);
    params.max_limit = static_cast<double>(value[MAX_LIMIT]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("Failed to load required parameter(s) for pass through filter: '%s'",
                            ex.getMessage().c_str());
    return false;
  }

  if (value.hasMember(NEGATIVE))
  {
    // Optional parameter(s)
    try
    {
      params.negative = static_cast<bool>(value[NEGATIVE]);
    }
    catch (const XmlRpc::XmlRpcException& ex)
    {
      CONSOLE_BRIDGE_logWarn("Failed to load optional parameter(s) for pass through filter: '%s'",
                             ex.getMessage().c_str());
    }
  }

  return true;
}

template <typename PointT>
bool PassThroughFilter<PointT>::filter(const T& input, T& output)
{
  // Create a shared pointer to the input object with a "destructor" function that does not delete the raw pointer
  auto cloud = boost::shared_ptr<const T>(&input, [](const T*) {});

  pcl::PassThrough<PointT> f;

  // Set the parameters
  if (!params.filter_field_name.empty())
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

  f.setInputCloud(cloud);
  f.filter(output);

  return true;
}

template <typename PointT>
std::string PassThroughFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(this)>();
}

}  // namespace cloud
}  // namespace noether_filtering

#define PCL_INSTANTIATE_PassThroughFilter(T) template class PCL_EXPORTS noether_filtering::cloud::PassThroughFilter<T>;

#endif  // NOETHER_FILTERING_CLOUD_IMPL_PASS_THROUGH_FILTER_HPP
