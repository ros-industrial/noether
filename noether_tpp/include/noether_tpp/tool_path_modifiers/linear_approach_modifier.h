#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Adds a series of waypoints in a linear pattern off the first waypoint in a tool path
 */
class LinearApproachModifier : public ToolPathModifier
{
public:
  enum class Axis
  {
    X,
    Y,
    Z
  };

  static Eigen::Vector3d toVector(double offset, Axis axis);

  LinearApproachModifier(Eigen::Vector3d offset, std::size_t n_points);
  LinearApproachModifier(double offset_height, Axis axis, std::size_t n_points);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  Eigen::Vector3d offset_;
  const size_t n_points_;
};

}  // namespace noether
