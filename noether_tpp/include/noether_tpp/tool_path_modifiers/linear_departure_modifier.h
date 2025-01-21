#pragma once

#include <noether_tpp/tool_path_modifiers/linear_approach_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Adds a series of waypoints in a linear pattern off the last waypoint in a tool path
 */
class LinearDepartureModifier : public LinearApproachModifier
{
public:
  using LinearApproachModifier::LinearApproachModifier;

  ToolPaths modify(ToolPaths tool_paths) const override final;
};

}  // namespace noether
