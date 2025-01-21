#pragma once

#include <memory>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
/**
 * @ingroup origin_generators
 * @brief Returns the center of the axis-aligned bounding box (AABB) as the origin
 */
struct AABBCenterOriginGenerator : public OriginGenerator
{
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;
};

}  // namespace noether
