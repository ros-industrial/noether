#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Aligns the orientation of each waypoint with the existing waypoint normal (z-axis) and the specified reference
 * x-axis direction
 * @details The new waypoint y-axis is computed as the cross-product of the existing waypoint normal (z-axis) and the
 * reference x-axis. The new waypoint x-axis is computed as the cross-product of the new y-axis with the existing
 * waypoint normal (z-axis). Thus, each new waypoint's x-axis will probably not exactly match the input reference
 * direction, but it will be as close as possible
 */
class AngledOrientationModifier : public ToolPathModifier
{
public:
  AngledOrientationModifier(double angle_offset, double tool_radius, bool flip_sign);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double angle_offset_;
  const double tool_radius_;
  const bool flip_sign_;
};

}  // namespace noether

