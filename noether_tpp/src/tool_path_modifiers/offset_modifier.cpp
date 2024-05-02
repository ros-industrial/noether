#include <noether_tpp/tool_path_modifiers/offset_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
OffsetModifier::OffsetModifier(Eigen::Isometry3d offset) : offset_(offset) {}

ToolPaths OffsetModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tp : tool_paths)
  {
    for (ToolPathSegment& tps : tp)
    {
      for (Eigen::Isometry3d& w : tps)
      {
        w = offset_ * w;
      }
    }
  }

  return tool_paths;
}

}  // namespace noether
