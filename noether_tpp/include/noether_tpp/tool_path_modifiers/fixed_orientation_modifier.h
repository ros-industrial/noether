#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Aligns the orientation of each waypoint with the existing waypoint normal (z-axis) and the specified reference
 * x-axis direction
 * @details The new waypoint y-axis is computed as the cross-product of the existing waypoint normal (z-axis) and the
 * reference x-axis. The new waypoint x-axis is computed as the cross-product of the new y-axis with the existing
 * waypoint normal (z-axis). Thus, each new waypoint's x-axis will probably not exactly match the input reference
 * direction, but it will be as close as possible
 */
class FixedOrientationModifier : public OneTimeToolPathModifier
{
public:
  FixedOrientationModifier(const Eigen::Vector3d& reference_x_direction);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  /** @brief Reference x-axis direction */
  const Eigen::Vector3d ref_x_dir_;
};

}  // namespace noether
