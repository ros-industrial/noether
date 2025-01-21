#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief The UniformSpacingLinearModifier class
 */
class UniformSpacingLinearModifier : public ToolPathModifier
{
public:
  UniformSpacingLinearModifier(const double point_spacing);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  double point_spacing_;
};

}  // namespace noether
