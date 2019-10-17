#ifndef NOETHER_FILTERING_CLOUD_PASS_THROUGH_FILTER_H
#define NOETHER_FILTERING_CLOUD_PASS_THROUGH_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>

namespace noether_filtering
{
template<typename PointT>
class PassThroughFilter : public FilterBase<typename pcl::PointCloud<PointT>::Ptr>
{
public:
  using T = typename pcl::PointCloud<PointT>::Ptr;

  using FilterBase<T>::FilterBase;

  virtual bool configure(XmlRpc::XmlRpcValue config) final;
  virtual bool filter(const T &input, T &output) final;
  virtual std::string getName() const final;

  struct Params
  {
    std::string filter_field_name = "x";
    float min_limit = -std::numeric_limits<float>::max();
    float max_limit = std::numeric_limits<float>::max();
    bool negative = false;
  };
  Params params;
};

} // namespace noether_filtering

#endif // NOETHER_FILTERING_CLOUD_PASS_THROUGH_FILTER_H
