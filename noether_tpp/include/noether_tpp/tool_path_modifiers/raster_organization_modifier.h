#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Organizes a tool path into a raster-style pattern
 * @details  Waypoints are sorted within each tool path segment in ascending order by x-axis value. Tool path segments
 * are sorted within each tool path in ascending order by the x-axis value of their first waypoint. Tool paths are
 * sorted in ascending order by the y-axis value of the first waypoint in the first tool path segment.
 */
struct RasterOrganizationModifier : ToolPathModifier
{
  using ToolPathModifier::ToolPathModifier;

  ToolPaths modify(ToolPaths) const override;
};

}  // namespace noether
