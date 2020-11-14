#ifndef NOETHER_FILTERING_CLOUD_MLS_SMOOTHING_FILTER_H
#define NOETHER_FILTERING_CLOUD_MLS_SMOOTHING_FILTER_H

#include "noether_filtering/filter_base.h"

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
class MLSSmoothingFilter : public FilterBase<pcl::PointCloud<PointT>>
{
public:
  using T = typename pcl::PointCloud<PointT>;

  using FilterBase<T>::FilterBase;

  /**
   * @brief configure
   * @param config: XmlRpc value
   * - config:
   *     polynomial_order:
   *     search_radius:
   * @return
   */
  bool configure(XmlRpc::XmlRpcValue config) override final;
  bool filter(const T& input, T& output) override final;
  std::string getName() const override final;

  struct Params
  {
    int polynomial_order = 2;
    double search_radius = 0.1;
  };
  Params params;

  static const std::string POLYNOMIAL_ORDER;
  static const std::string SEARCH_RADIUS;
};

}  // namespace cloud
}  // namespace noether_filtering

#endif  // NOETHER_FILTERING_CLOUD_MLS_SMOOTHING_FILTER_H
