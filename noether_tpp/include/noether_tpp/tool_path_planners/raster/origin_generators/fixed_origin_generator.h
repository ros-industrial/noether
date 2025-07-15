#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

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

protected:
  Eigen::Vector3d origin_;

  DECLARE_YAML_FRIEND_CLASSES(FixedOriginGenerator)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::FixedOriginGenerator)
