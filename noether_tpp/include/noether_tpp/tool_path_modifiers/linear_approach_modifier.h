#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Adds a series of waypoints in a linear pattern off the first waypoint in a tool path
 */
class LinearApproachModifier : public ToolPathModifier
{
public:

  LinearApproachModifier(Eigen::Vector3d offset, std::size_t n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  Eigen::Vector3d offset_;
  const size_t n_points_;
};

}  // namespace noether
