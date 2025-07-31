#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief The UniformSpacingSplineModifier class
 */
class UniformSpacingSplineModifier : public ToolPathModifier
{
public:
  UniformSpacingSplineModifier(const double point_spacing);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  double point_spacing_;

  UniformSpacingSplineModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(UniformSpacingSplineModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::UniformSpacingSplineModifier)
