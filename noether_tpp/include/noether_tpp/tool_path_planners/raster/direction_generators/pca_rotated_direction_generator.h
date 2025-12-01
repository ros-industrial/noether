#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
/**
 * @ingroup direction_generators
 * @brief Rotates a direction vector provided by another direction generator about the smallest principal axis of the
 * input mesh
 * @details One use for this class is for cross-hatch tool paths where the nominal direction is provided by a given
 * direction generator, and this direction generator provides the rotated direction for the cross pattern
 */
class PCARotatedDirectionGenerator : public DirectionGenerator
{
public:
  PCARotatedDirectionGenerator(const PCARotatedDirectionGenerator&) = delete;
  PCARotatedDirectionGenerator(PCARotatedDirectionGenerator&&) = delete;

  PCARotatedDirectionGenerator& operator=(const PCARotatedDirectionGenerator&) = delete;
  PCARotatedDirectionGenerator& operator=(PCARotatedDirectionGenerator&&) = delete;

  PCARotatedDirectionGenerator(DirectionGenerator::ConstPtr&& dir_gen, double rotation_angle);

  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override;

protected:
  const DirectionGenerator::ConstPtr dir_gen_;
  const double rotation_angle_;
};

}  // namespace noether
