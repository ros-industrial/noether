#pragma once

#include <noether_tpp/tool_path_modifiers/linear_approach_modifier.h>
#include <noether_tpp/macros.h>

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

protected:
  DECLARE_YAML_FRIEND_CLASSES(LinearDepartureModifier);
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::LinearDepartureModifier)
