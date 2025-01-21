#pragma once

#include <memory>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
/**
 * @ingroup origin_generators
 * @brief Returns the origin as the origin of another specified origin generator plus a fixed offset
 */
class OffsetOriginGenerator : public OriginGenerator
{
public:
  OffsetOriginGenerator(const OffsetOriginGenerator&) = delete;
  OffsetOriginGenerator(OffsetOriginGenerator&&) = delete;

  OffsetOriginGenerator& operator=(const OffsetOriginGenerator&) = delete;
  OffsetOriginGenerator& operator=(OffsetOriginGenerator&&) = delete;

  OffsetOriginGenerator(OriginGenerator::ConstPtr&& generator, const Eigen::Vector3d& offset);
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;

private:
  const OriginGenerator::ConstPtr generator_;
  const Eigen::Vector3d offset_;
};

}  // namespace noether
