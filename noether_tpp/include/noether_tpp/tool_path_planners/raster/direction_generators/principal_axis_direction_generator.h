#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

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

protected:
  /**
   * @brief Rotation offset (radians) to apply about the smallest principal axis to the generated raster direction
   */
  double rotation_offset_;

  DECLARE_YAML_FRIEND_CLASSES(PrincipalAxisDirectionGenerator)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::PrincipalAxisDirectionGenerator)
