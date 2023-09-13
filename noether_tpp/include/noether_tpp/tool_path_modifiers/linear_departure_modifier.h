#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Modifier that adjusts the parameters of the tool departure trajectory to the media
 */
class LinearDepartureModifier : public ToolPathModifier
{
public:
  LinearDepartureModifier(Eigen::Vector3d offset_, double n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  Eigen::Vector3d offset_;
  const double n_points_;
};

}  // namespace noether

