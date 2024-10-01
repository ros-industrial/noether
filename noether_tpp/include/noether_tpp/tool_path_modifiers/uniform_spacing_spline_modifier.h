#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
class UniformSpacingSplineModifier : public ToolPathModifier
{
public:
  UniformSpacingSplineModifier(const double point_spacing);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  double point_spacing_;
};

}  // namespace noether
