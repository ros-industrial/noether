#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup direction_generators
 * @brief Generates the raster direction based on a fixed input
 */
class FixedDirectionGenerator : public DirectionGenerator
{
public:
  FixedDirectionGenerator(const Eigen::Vector3d& direction);
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;

protected:
  Eigen::Vector3d direction_;

  FixedDirectionGenerator() = default;
  DECLARE_YAML_FRIEND_CLASSES(FixedDirectionGenerator)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::FixedDirectionGenerator)
