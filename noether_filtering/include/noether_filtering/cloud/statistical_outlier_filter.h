#ifndef STATISTICAL_OUTLIER_FILTER_H
#define STATISTICAL_OUTLIER_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>

namespace noether_filtering
{
template<typename PointT>
class StatisticalOutlierFilter : public FilterBase<typename pcl::PointCloud<PointT>::Ptr>
{
public:
  typedef typename pcl::PointCloud<PointT>::Ptr T;
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
  virtual bool filter(const T &input, T &output) override final;
  virtual std::string getName() const override final;

  struct Params
  {
    int mean_k = 10;
    double std_dev_mult = 1.0;
  };
  Params params;
};

} // namespace noether_filtering

#endif // STATISTICAL_OUTLIER_FILTER_H
