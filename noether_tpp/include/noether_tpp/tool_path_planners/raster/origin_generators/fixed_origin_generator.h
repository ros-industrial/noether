#pragma once

#include <memory>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
/**
 * @ingroup origin_generators
 * @brief Returns a fixed point as the origin
 * @details The default fixed point is a zero vector corresponding to the mesh coordinate system origin
 */
class FixedOriginGenerator : public OriginGenerator
{
public:
  FixedOriginGenerator(const Eigen::Vector3d& origin = Eigen::Vector3d::Zero());
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;

private:
  const Eigen::Vector3d origin_;
};

}  // namespace noether
