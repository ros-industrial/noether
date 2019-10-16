#ifndef STATISTICAL_OUTLIER_FILTER_HPP
#define STATISTICAL_OUTLIER_FILTER_HPP

#include "noether_filtering/cloud/statistical_outlier_filter.h"
#include <XmlRpcException.h>
#include <console_bridge/console.h>

namespace noether_filtering
{
template<typename PointT>
bool StatisticalOutlierFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  try
  {
    params_.mean_k = static_cast<int>(value["mean_k"]);
    params_.std_dev_mult = static_cast<double>(value["std_dev_mult"]);
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    CONSOLE_BRIDGE_logError("Error configuring statistical outlier filter: '%s'",
                            ex.getMessage().c_str());
    return false;
  }
  return true;
}

template<typename PointT>
bool StatisticalOutlierFilter<PointT>::filter(const T &input, T &output)
{
  output.reset(new pcl::PointCloud<PointT>());

  // Apply the parameters
  filter_.setMeanK(params_.mean_k);
  filter_.setStddevMulThresh(params_.std_dev_mult);

  filter_.setInputCloud(input);
  filter_.filter(*output);

  return true;
}

} // namespace noether_filtering

#define PCL_INSTANTIATE_StatisticalOutlierFilter(T) \
  template class PCL_EXPORTS noether_filtering::StatisticalOutlierFilter<T>;

#endif // STATISTICAL_OUTLIER_FILTER_HPP
