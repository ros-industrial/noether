#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Organizes a set of tool paths into a standard configuration for edge paths
 * @details Segments are ordered in a tool path such that the start of one segment is as close as
 * possible to the end of the previous segment. The first segment is chosen as the one whose first waypoint is closest
 * to a reference position. Tool path segments are ordered in the top-level container in order of length, from longest
 * to shortest. Note: waypoints are expected to be ordered correctly on input to the modifier
 */
class StandardEdgePathsOrganizationModifier : public OneTimeToolPathModifier
{
public:
  StandardEdgePathsOrganizationModifier(const Eigen::Vector3d& start_reference = Eigen::Vector3d::Zero());

  ToolPaths modify(ToolPaths) const override;

private:
  const Eigen::Vector3d start_reference_;
};

}  // namespace noether
