#ifndef NOETHER_FILTERING_CLOUD_IMPL_RADIUS_OUTLIER_FILTER_HPP
#define NOETHER_FILTERING_CLOUD_IMPL_RADIUS_OUTLIER_FILTER_HPP

#include "noether_filtering/cloud/radius_outlier_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <XmlRpcException.h>
#include <console_bridge/console.h>

namespace noether_filtering
{
template<typename PointT>
bool RadiusOutlierFilter<PointT>::configure(XmlRpc::XmlRpcValue config)
{
  try
  {
    params.radius = static_cast<double>(config["radius"]);
    params.min_pts = static_cast<int>(config["min_pts"]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("Error configuring radius outlier filter: '%s'",
                            ex.getMessage().c_str());
    return false;
  }
  return true;
}

template<typename PointT>
bool RadiusOutlierFilter<PointT>::filter(const T &input, T &output)
{
  output.reset(new pcl::PointCloud<PointT>());

  pcl::RadiusOutlierRemoval<PointT> f;

  // Set the parameters
  f.setRadiusSearch(params.radius);
  f.setMinNeighborsInRadius(params.min_pts);

  f.setInputCloud(input);
  f.filter(*output);

  return true;
}

template<typename PointT>
std::string RadiusOutlierFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(this)>();
}

} // namespace noether_filtering

#define PCL_INSTANTIATE_RadiusOutlierFilter(T) \
  template class PCL_EXPORTS noether_filtering::RadiusOutlierFilter<T>;

#endif // NOETHER_FILTERING_CLOUD_IMPL_RADIUS_OUTLIER_FILTER_HPP
