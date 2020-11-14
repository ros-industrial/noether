#ifndef STATISTICAL_OUTLIER_FILTER_HPP
#define STATISTICAL_OUTLIER_FILTER_HPP

#include <console_bridge/console.h>
#include "noether_filtering/cloud/statistical_outlier_filter.h"
#include "noether_filtering/utils.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <XmlRpcException.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
const std::string StatisticalOutlierFilter<PointT>::MEAN_K = "mean_k";

template <typename PointT>
const std::string StatisticalOutlierFilter<PointT>::STD_DEV_MULT = "std_dev_mult";

template <typename PointT>
bool StatisticalOutlierFilter<PointT>::configure(XmlRpc::XmlRpcValue value)
{
  std::string error;
  if (!value.hasMember(MEAN_K))
    error += MEAN_K + ", ";
  if (!value.hasMember(STD_DEV_MULT))
    error += STD_DEV_MULT + ", ";

  if (!error.empty())
  {
    CONSOLE_BRIDGE_logError("Statistical outlier filter missing required parameters: %s", error.c_str());
    return false;
  }

  try
  {
    params.mean_k = static_cast<int>(value[MEAN_K]);
    params.std_dev_mult = static_cast<double>(value[STD_DEV_MULT]);
  }
  catch (const XmlRpc::XmlRpcException& ex)
  {
    CONSOLE_BRIDGE_logError("Error configuring statistical outlier filter: '%s'", ex.getMessage().c_str());
    return false;
  }
  return true;
}

template <typename PointT>
bool StatisticalOutlierFilter<PointT>::filter(const T& input, T& output)
{
  // Create a shared pointer to the input object with a "destructor" function that does not delete the raw pointer
  auto cloud = boost::shared_ptr<const T>(&input, [](const T*) {});

  pcl::StatisticalOutlierRemoval<PointT> f;

  // Apply the parameters
  f.setMeanK(params.mean_k);
  f.setStddevMulThresh(params.std_dev_mult);

  f.setInputCloud(cloud);
  f.filter(output);

  return true;
}

template <typename PointT>
std::string StatisticalOutlierFilter<PointT>::getName() const
{
  return utils::getClassName<decltype(this)>();
}

}  // namespace cloud
}  // namespace noether_filtering

#define PCL_INSTANTIATE_StatisticalOutlierFilter(T)                                                                    \
  template class PCL_EXPORTS noether_filtering::cloud::StatisticalOutlierFilter<T>;

#endif  // STATISTICAL_OUTLIER_FILTER_HPP
