#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Aligns the x-axis of all waypoints with the direction of travel of the first two waypoints
 * @details Re-alignment of the waypoint orientation only occurs if the dot product of the waypoint x-axis and the
 * reference direction of travel is less than zero. In the case of an orientation change, the waypoint is simply rotated
 * about its z-axis by 180 degrees
 */
struct UniformOrientationModifier : public OneTimeToolPathModifier
{
  ToolPaths modify(ToolPaths tool_paths) const override final;
};

}  // namespace noether
