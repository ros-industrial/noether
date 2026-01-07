#include <noether_tpp/tool_path_modifiers/join_close_segments_modifier.h>
#include <noether_tpp/serialization.h>

namespace noether
{
JoinCloseSegmentsToolPathModifier::JoinCloseSegmentsToolPathModifier(const double distance)
  : OneTimeToolPathModifier(), distance_(distance)
{
}

ToolPaths JoinCloseSegmentsToolPathModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    if (tool_path.size() < 2)
      continue;

    auto segment = tool_path.begin();
    auto next_segment = segment + 1;

    while (next_segment != tool_path.end())
    {
      const double d = (segment->back().translation() - next_segment->front().translation()).norm();
      if (d < distance_)
      {
        // Combine and remove
        segment->insert(segment->end(), next_segment->begin(), next_segment->end());
        next_segment = tool_path.erase(next_segment);
      }
      else
      {
        ++segment;
        ++next_segment;
      }
    }
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::JoinCloseSegmentsToolPathModifier>::encode(const noether::JoinCloseSegmentsToolPathModifier& val)
{
  Node node;
  node["distance"] = val.distance_;
  return node;
}

bool convert<noether::JoinCloseSegmentsToolPathModifier>::decode(const Node& node,
                                                                 noether::JoinCloseSegmentsToolPathModifier& val)
{
  val.distance_ = getMember<double>(node, "distance");
  return true;
}
/** @endcond */
}  // namespace YAML
