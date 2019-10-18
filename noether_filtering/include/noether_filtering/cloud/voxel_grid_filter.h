#ifndef NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H
#define NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/point_cloud.h>

namespace noether_filtering
{
template<typename PointT>
class VoxelGridFilter : public FilterBase<typename pcl::PointCloud<PointT>::Ptr>
{
public:
  using T = typename pcl::PointCloud<PointT>::Ptr;

  using FilterBase<T>::FilterBase;

  bool configure(XmlRpc::XmlRpcValue config) override final;
  bool filter(const T &input, T &output) override final;
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
};

} // namespace noether_filtering

#endif // NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H
