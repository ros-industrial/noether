#pragma once

#include <memory>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
/**
 * @ingroup origin_generators
 * @brief Returns the centroid of the mesh vertices as the origin
 */
struct CentroidOriginGenerator : public OriginGenerator
{
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;
};

}  // namespace noether
