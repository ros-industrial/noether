#include <noether_tpp/tool_path_modifiers/offset_modifier.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/serialization.h>

namespace noether
{
OffsetModifier::OffsetModifier(const Eigen::Isometry3d& offset) : offset_(offset) {}

ToolPaths OffsetModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tp : tool_paths)
  {
    for (ToolPathSegment& tps : tp)
    {
      for (Eigen::Isometry3d& w : tps)
      {
        w = w * offset_;
      }
    }
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::OffsetModifier>::encode(const noether::OffsetModifier& val)
{
  Node node;
  node["offset"] = val.offset_;
  return node;
}

bool convert<noether::OffsetModifier>::decode(const Node& node, noether::OffsetModifier& val)
{
  val.offset_ = getMember<Eigen::Isometry3d>(node, "offset");
  return true;
}
/** @endcond */

}  // namespace YAML
