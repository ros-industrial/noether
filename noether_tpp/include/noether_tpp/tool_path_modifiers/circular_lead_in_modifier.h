#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Modifier that adjusts the parameters of the tool approach trajectory to the media
 */
class CircularLeadInModifier : public ToolPathModifier
{
public:
  CircularLeadInModifier(double arc_angle, double arc_radius, double n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double arc_angle_;
  const double arc_radius_;
  const double n_points_;
};

}  // namespace noether
