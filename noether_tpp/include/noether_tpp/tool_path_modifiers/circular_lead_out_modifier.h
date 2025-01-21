#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Modifier that adds exit waypoints in a circular arc (with fixed orientation) to the end of a trajectory
 */
class CircularLeadOutModifier : public ToolPathModifier
{
public:
  CircularLeadOutModifier(double arc_angle, double arc_radius, double n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double arc_angle_;
  const double arc_radius_;
  const double n_points_;
};

}  // namespace noether
