#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
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

}  // namespace noether
