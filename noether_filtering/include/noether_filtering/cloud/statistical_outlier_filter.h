#ifndef STATISTICAL_OUTLIER_FILTER_H
#define STATISTICAL_OUTLIER_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
class StatisticalOutlierFilter : public FilterBase<pcl::PointCloud<PointT>>
{
public:
  using T = pcl::PointCloud<PointT>;

  using FilterBase<T>::FilterBase;

  /**
   * @brief configure
   * @param config: XmlRpc value
   * - config:
   *     mean_k: (int)
   *     std_dev_mult: (double)
   * @return
   */
  virtual bool configure(XmlRpc::XmlRpcValue config) override final;
  virtual bool filter(const T& input, T& output) override final;
  virtual std::string getName() const override final;

  struct Params
  {
    int mean_k = 10;
    double std_dev_mult = 1.0;
  };
  Params params;

  static const std::string MEAN_K;
  static const std::string STD_DEV_MULT;
};
}  // namespace cloud
}  // namespace noether_filtering

#endif  // STATISTICAL_OUTLIER_FILTER_H
