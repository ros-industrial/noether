#ifndef NOETHER_FILTERING_CLOUD_RADIUS_OUTLIER_FILTER_H
#define NOETHER_FILTERING_CLOUD_RADIUS_OUTLIER_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
class RadiusOutlierFilter : public FilterBase<pcl::PointCloud<PointT>>
{
public:
  using T = pcl::PointCloud<PointT>;

  using FilterBase<T>::FilterBase;

  /**
   * @brief configure
   * @param config: XmlRpc value
   * - config:
   *     radius: (double)
   *     min_pts: (int)
   * @return
   */
  virtual bool configure(XmlRpc::XmlRpcValue config) override final;
  virtual bool filter(const T& input, T& output) override final;
  virtual std::string getName() const override final;

  struct Params
  {
    double radius = 0.1;
    int min_pts = 10;
  };
  Params params;

  static const std::string RADIUS;
  static const std::string MIN_PTS;
};
}  // namespace cloud
}  // namespace noether_filtering

#endif  // NOETHER_FILTERING_CLOUD_RADIUS_OUTLIER_FILTER_H
