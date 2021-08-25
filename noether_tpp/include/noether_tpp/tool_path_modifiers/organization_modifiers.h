#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Organizes a tool path into a raster-style pattern
 * @details The tool path is organized with the following structure:
 *   - waypoints are sorted within each tool path segment in ascending order by x-axis value
 *   - tool path segments are sorted within each tool path in ascending order by the x-axis value of their first
 * waypoint
 *   - tool paths are sorted in ascending order by the y-axis value of the first waypoint in the first tool path segment
 */
struct RasterOrganizationModifier : OneTimeToolPathModifier
{
  using OneTimeToolPathModifier::OneTimeToolPathModifier;

  ToolPaths modify(ToolPaths) const override;
};

/**
 * @brief Organizes a tool path into a snake-style pattern
 * @details The tool path is organized with the following structure:
 *   - Tool paths at even indices are sorted by ascending waypoint x-axis value; segments in the tool path are also
 * sorted by ascending x-axis value of their first waypoint
 *   - Tool paths at odd indices are sorted by descending waypoint x-axis value; segments in the tool path are also
 * sorted by descending x-axis value of their first waypoint
 *   - Tool paths are sorted relative to one another by ascending y-axis value of the first waypoint in the first tool
 * path segment
 */
struct SnakeOrganizationModifier : OneTimeToolPathModifier
{
  using OneTimeToolPathModifier::OneTimeToolPathModifier;

  ToolPaths modify(ToolPaths) const override;
};

}  // namespace noether
