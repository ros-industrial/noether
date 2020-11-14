#ifndef NOETHER_FILTERING_CLOUD_IMPL_RADIUS_OUTLIER_FILTER_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_RADIUS_OUTLIER_FILTER_HPP

#include "noether_filtering/cloud/radius_outlier_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <XmlRpcException.h>
#include <console_bridge/console.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
const std::string RadiusOutlierFilter<PointT>::RADIUS = "radius";

template <typename PointT>
const std::string RadiusOutlierFilter<PointT>::MIN_PTS = "min_pts";

template <typename PointT>
bool RadiusOutlierFilter<PointT>::configure(XmlRpc::XmlRpcValue config)
{
  std::string error;
  if (!config.hasMember("radius"))
    error += RADIUS + ", ";
  if (!config.hasMember("min_pts"))
    error += MIN_PTS + ", ";

  if (!error.empty())
  {
    CONSOLE_BRIDGE_logError("Radius outlier filter configuration missing parameters: %s", error.c_str());
    return false;
  }

  try
  {
    params.radius = static_cast<double>(config[RADIUS]);
    params.min_pts = static_cast<int>(config[MIN_PTS]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("Error configuring radius outlier filter: '%s'", ex.getMessage().c_str());
    return false;
  }
  return true;
}

template <typename PointT>
bool RadiusOutlierFilter<PointT>::filter(const T& input, T& output)
{
  // Create a shared pointer to the input object with a "destructor" function that does not delete the raw pointer
  auto cloud = boost::shared_ptr<const T>(&input, [](const T*) {});

  pcl::RadiusOutlierRemoval<PointT> f;

  // Set the parameters
  f.setRadiusSearch(params.radius);
  f.setMinNeighborsInRadius(params.min_pts);

  f.setInputCloud(cloud);
  f.filter(output);

  return true;
}

template <typename PointT>
std::string RadiusOutlierFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(this)>();
}

}  // namespace cloud
}  // namespace noether_filtering

#define PCL_INSTANTIATE_RadiusOutlierFilter(T)                                                                         \
  template class PCL_EXPORTS noether_filtering::cloud::RadiusOutlierFilter<T>;

#endif  // NOETHER_FILTERING_CLOUD_IMPL_RADIUS_OUTLIER_FILTER_HPP
