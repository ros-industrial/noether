#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
class PointSpacingModifier : public ToolPathModifier
{
public:
  PointSpacingModifier(const double point_spacing);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  double point_spacing_;
};

}  // namespace noether
