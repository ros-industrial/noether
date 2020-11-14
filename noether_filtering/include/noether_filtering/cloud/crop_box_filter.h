#ifndef NOETHER_FILTERING_CLOUD_CROP_BOX_FILTER_H
#define NOETHER_FILTERING_CLOUD_CROP_BOX_FILTER_H

#include "noether_filtering/filter_base.h"
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>

namespace noether_filtering
{
namespace cloud
{
template <typename PointT>
class CropBoxFilter : public FilterBase<pcl::PointCloud<PointT>>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using T = typename pcl::PointCloud<PointT>;

  using FilterBase<T>::FilterBase;

  /**
   * @brief configure
   * @param config: XmlRpc value
   * - config:
   *     min_pt:
   *       x: (double)
   *       y: (double)
   *       z: (double)
   *     max_pt:
   *       x: (double)
   *       y: (double)
   *       z: (double)
   *     transform:
   *       x: (double)
   *       y: (double)
   *       z: (double)
   *       rx: (double)
   *       ry: (double)
   *       rz: (double)
   *     crop_outside: (bool)
   * @return
   */
  bool configure(XmlRpc::XmlRpcValue config) override final;
  bool filter(const T& input, T& output) override final;
  std::string getName() const override final;

  struct Params
  {
    Eigen::Vector4f min_pt = -1.0 * Eigen::Vector4f::Ones();
    Eigen::Vector4f max_pt = Eigen::Vector4f::Ones();
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    bool crop_outside = false;
  };
  Params params;

  static const std::string MAX;
  static const std::string MIN;
  static const std::string TRANSFORM;
  static const std::string CROP_OUTSIDE;
};

}  // namespace cloud
}  // namespace noether_filtering

#endif  // NOETHER_FILTERING_CLOUD_CROP_BOX_FILTER_H
