#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Aligns the x-axis of all waypoints with the direction of travel between adjacent waypoints
 */
struct DirectionOfTravelOrientationModifier : public OneTimeToolPathModifier
{
  ToolPaths modify(ToolPaths tool_paths) const override final;
};

}  // namespace noether
