#ifndef STATISTICAL_OUTLIER_FILTER_HPP
#define STATISTICAL_OUTLIER_FILTER_HPP

#include <console_bridge/console.h>
#include "noether_filtering/cloud/statistical_outlier_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <XmlRpcException.h>

namespace noether_filtering
{
template<typename PointT>
bool StatisticalOutlierFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  try
  {
    params.mean_k = static_cast<int>(value["mean_k"]);
    params.std_dev_mult = static_cast<double>(value["std_dev_mult"]);
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

  pcl::StatisticalOutlierRemoval<PointT> f;

  // Apply the parameters
  f.setMeanK(params.mean_k);
  f.setStddevMulThresh(params.std_dev_mult);

  f.setInputCloud(input);
  f.filter(*output);

  return true;
}

template<typename PointT>
std::string StatisticalOutlierFilter<PointT>::getName()
{
  return utils::getClassName<decltype(this)>();
}

} // namespace noether_filtering

#define PCL_INSTANTIATE_StatisticalOutlierFilter(T) \
  template class PCL_EXPORTS noether_filtering::StatisticalOutlierFilter<T>;

#endif // STATISTICAL_OUTLIER_FILTER_HPP
