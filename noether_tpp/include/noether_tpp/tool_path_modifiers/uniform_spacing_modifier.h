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
class UniformSpacingModifier : public ToolPathModifier
{
public:
  UniformSpacingModifier(const double point_spacing, const long spline_degree);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  double point_spacing_;
  long spline_degree_;

  UniformSpacingModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(UniformSpacingModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::UniformSpacingModifier)
