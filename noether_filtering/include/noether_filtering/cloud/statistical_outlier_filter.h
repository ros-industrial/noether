#ifndef STATISTICAL_OUTLIER_FILTER_H
#define STATISTICAL_OUTLIER_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace noether_filtering
{
template<typename PointT>
class StatisticalOutlierFilter : public FilterBase<typename pcl::PointCloud<PointT>::Ptr>
{
public:
  struct Params
  {
    int mean_k = 10;
    double std_dev_mult = 1.0;
  };

  typedef typename pcl::PointCloud<PointT>::Ptr T;
  using FilterBase<T>::FilterBase;

  virtual bool configure(XmlRpc::XmlRpcValue config) final;
  virtual bool filter(const T &input, T &output) final;
  inline std::string getName() final
  {
    return "statistical_outlier_removal";
  }

protected:
  pcl::StatisticalOutlierRemoval<PointT> filter_;
  Params params_;
};

} // namespace noether_filtering

#endif // STATISTICAL_OUTLIER_FILTER_H
