#ifndef NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H
#define NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H

#include "noether_filtering/filter_base.h"
#include <pcl/filters/voxel_grid.h>

namespace noether_filtering
{
template<typename PointT>
class VoxelGridFilter : public FilterBase<typename pcl::PointCloud<PointT>::Ptr>
{
public:
  struct Params
  {
    double leaf_size = 0.01;
    double min_limit = -std::numeric_limits<double>::max();
    double max_limit = std::numeric_limits<double>::max();
    bool filter_limits_negative = false;
    unsigned int min_pts_per_voxel = 1;
    std::string filter_field_name = "";
  };

  using T = typename pcl::PointCloud<PointT>::Ptr;

  using FilterBase<T>::FilterBase;

  virtual bool configure(XmlRpc::XmlRpcValue config) final;
  virtual bool filter(const T &input, T &output) final;
  inline std::string getName() final
  {
    return "voxel_grid";
  }

private:
  pcl::VoxelGrid<PointT> filter_;
  Params params_;
};

} // namespace noether_filtering

#endif // NOETHER_FILTERING_CLOUD_VOXEL_GRID_FILTER_H
