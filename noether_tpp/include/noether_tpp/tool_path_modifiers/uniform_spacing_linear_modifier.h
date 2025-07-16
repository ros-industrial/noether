#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

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

  UniformSpacingLinearModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(UniformSpacingLinearModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::UniformSpacingLinearModifier)
