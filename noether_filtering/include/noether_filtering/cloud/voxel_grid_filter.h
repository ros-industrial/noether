#ifndef NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H
#define NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
class VoxelGridFilter : public FilterBase<pcl::PointCloud<PointT>>
{
public:
  using T = pcl::PointCloud<PointT>;

  using FilterBase<T>::FilterBase;

  /**
   * @brief configure
   * @param config: XmlRpc value
   * - config:
   *     leaf_size: (string)
   *     min_limit: (double, optional)
   *     max_limit: (double, optional)
   *     filter_limits_negative: (bool, optional)
   *     min_pts_per_voxel: (int, optional)
   *     filter_field_name: (string, optional)
   * @return
   */
  bool configure(XmlRpc::XmlRpcValue config) override final;
  bool filter(const T& input, T& output) override final;
  std::string getName() const override final;

  struct Params
  {
    float leaf_size = 0.01;
    float min_limit = -std::numeric_limits<float>::max();
    float max_limit = std::numeric_limits<float>::max();
    bool filter_limits_negative = false;
    unsigned int min_pts_per_voxel = 1;
    std::string filter_field_name = "";
  };
  Params params;

  static const std::string LEAF_SIZE;
  static const std::string FILTER_FIELD_NAME;
  static const std::string MIN_LIMIT;
  static const std::string MAX_LIMIT;
  static const std::string FILTER_LIMITS_NEGATIVE;
  static const std::string MIN_PTS_PER_VOXEL;
};
}  // namespace cloud
}  // namespace noether_filtering

#endif  // NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H
