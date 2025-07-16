#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

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
  double arc_angle_;
  double arc_radius_;
  double n_points_;

  CircularLeadOutModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(CircularLeadOutModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::CircularLeadOutModifier)
