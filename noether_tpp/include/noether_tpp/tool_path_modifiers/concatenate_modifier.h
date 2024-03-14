#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Combines existing toolpaths into one toolpath
 * @details A RasterOrganizationModifier is first used to organize the tool paths into a raster pattern. The tool paths
 * are then looped through to create a single concatenated toolpath.
 */
struct ConcatenateModifier : OneTimeToolPathModifier
{
  using OneTimeToolPathModifier::OneTimeToolPathModifier;

  ToolPaths modify(ToolPaths) const override;
};

}  // namespace noether
