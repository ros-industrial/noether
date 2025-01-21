#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
/**
 * @ingroup direction_generators
 * @brief Generates the raster direction along the largest principal axis of the input mesh
 */
class PrincipalAxisDirectionGenerator : public DirectionGenerator
{
public:
  PrincipalAxisDirectionGenerator(double rotation_offset = 0.0);
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;

private:
  /**
   * @brief Rotation offset (radians) to apply about the smallest principal axis to the generated raster direction
   */
  const double rotation_offset_;
};

}  // namespace noether
