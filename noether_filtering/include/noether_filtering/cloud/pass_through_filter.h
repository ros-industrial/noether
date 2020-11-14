#ifndef NOETHER_FILTERING_CLOUD_PASS_THROUGH_FILTER_H
#define NOETHER_FILTERING_CLOUD_PASS_THROUGH_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
class PassThroughFilter : public FilterBase<pcl::PointCloud<PointT>>
{
public:
  using T = pcl::PointCloud<PointT>;

  using FilterBase<T>::FilterBase;

  /**
   * @brief configure
   * @param config: XmlRpc value
   * - config:
   *     filter_field_name: (string)
   *     min_limit: (double)
   *     max_limit: (double)
   *     negative: (bool)
   * @return
   */
  bool configure(XmlRpc::XmlRpcValue config) override final;
  bool filter(const T& input, T& output) override final;
  std::string getName() const override final;

  struct Params
  {
    std::string filter_field_name = "x";
    float min_limit = -std::numeric_limits<float>::max();
    float max_limit = std::numeric_limits<float>::max();
    bool negative = false;
  };
  Params params;

  static const std::string FILTER_FIELD_NAME;
  static const std::string MIN_LIMIT;
  static const std::string MAX_LIMIT;
  static const std::string NEGATIVE;
};
}  // namespace cloud
}  // namespace noether_filtering

#endif  // NOETHER_FILTERING_CLOUD_PASS_THROUGH_FILTER_H
