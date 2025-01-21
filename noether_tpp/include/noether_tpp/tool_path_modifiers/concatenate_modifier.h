#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Concatenates all input tool paths into a single tool path
 * @details This modifier changes the input tool paths vector (i.e., `noether::ToolPaths` with size `n`) into a single
 * tool path (i.e., `noether::ToolPaths` with size of 1). It does not modify the structure of the underlying tool path
 * segments.
 */
struct ConcatenateModifier : OneTimeToolPathModifier
{
  using OneTimeToolPathModifier::OneTimeToolPathModifier;

  ToolPaths modify(ToolPaths) const override;
};

}  // namespace noether
