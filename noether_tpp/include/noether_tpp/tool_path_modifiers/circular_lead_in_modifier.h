#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

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
  double arc_angle_;
  double arc_radius_;
  double n_points_;

  CircularLeadInModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(CircularLeadInModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::CircularLeadInModifier)
