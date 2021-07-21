#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether_tpp
{
/**
 * @brief A tool path modifier that does not change the input tool path
 */
class NoOpToolPathModifier : public OneTimeToolPathModifier
{
public:
  using OneTimeToolPathModifier::OneTimeToolPathModifier;

  inline ToolPaths modify(ToolPaths tool_paths) const override final { return tool_paths; };
};

}  // namespace noether_tpp
