#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Organizes a tool path into a snake-style pattern
 * @details The tool paths at odd indices are the reversed to produce the snake pattern. This modifier typically needs
 * to be run behind a modifier that can organize tool path segments properly (e.g., raster organization tool path
 * modifier)
 */
struct SnakeOrganizationModifier : ToolPathModifier
{
  using ToolPathModifier::ToolPathModifier;

  ToolPaths modify(ToolPaths) const override;
};

}  // namespace noether
