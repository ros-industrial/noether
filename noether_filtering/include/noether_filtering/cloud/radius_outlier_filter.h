#ifndef NOETHER_FILTERING_CLOUD_RADIUS_OUTLIER_FILTER_H
#define NOETHER_FILTERING_CLOUD_RADIUS_OUTLIER_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>

namespace noether_filtering
{
template<typename PointT>
class RadiusOutlierFilter : public FilterBase<typename pcl::PointCloud<PointT>::Ptr>
{
  public:
  typedef typename pcl::PointCloud<PointT>::Ptr T;

  using FilterBase<T>::FilterBase;

  virtual bool configure(XmlRpc::XmlRpcValue config) final;
  virtual bool filter(const T &input, T &output) final;
  virtual std::string getName() final;

  struct Params
  {
    double radius = 0.1;
    int min_pts = 10;
  };
  Params params;
};

} // namespace noether_filtering

#endif // NOETHER_FILTERING_CLOUD_RADIUS_OUTLIER_FILTER_H
