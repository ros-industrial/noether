#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
class SplineModifier : public ToolPathModifier
{
public:
  SplineModifier(const double point_spacing);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  double point_spacing_;
};

}  // namespace noether
