#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Organizes a tool path into a raster-style pattern
 * @details  Waypoints are sorted within each tool path segment in ascending order by x-axis value. Tool path segments
 * are sorted within each tool path in ascending order by the x-axis value of their first waypoint. Tool paths are
 * sorted in ascending order by the y-axis value of the first waypoint in the first tool path segment.
 */
struct RasterOrganizationModifier : OneTimeToolPathModifier
{
  using OneTimeToolPathModifier::OneTimeToolPathModifier;

  ToolPaths modify(ToolPaths) const override;
};

/**
 * @brief Organizes a tool path into a snake-style pattern
 * @details A RasterOrganizationModifier is first used to organize the tool paths into a raster pattern. The tool paths
 * at odd indices are then reversed to produce the snake pattern.
 */
struct SnakeOrganizationModifier : OneTimeToolPathModifier
{
  using OneTimeToolPathModifier::OneTimeToolPathModifier;

  ToolPaths modify(ToolPaths) const override;
};

/**
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
