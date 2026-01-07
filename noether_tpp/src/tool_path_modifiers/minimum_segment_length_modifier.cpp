#include <noether_tpp/tool_path_modifiers/minimum_segment_length_modifier.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

namespace noether
{
MinimumSegmentLengthToolPathModifier::MinimumSegmentLengthToolPathModifier(const double minimum_length)
  : OneTimeToolPathModifier(), minimum_length_(minimum_length)
{
}

ToolPaths MinimumSegmentLengthToolPathModifier::modify(ToolPaths tool_paths) const
{
  auto tool_path = tool_paths.begin();
  while (tool_path != tool_paths.end())
  {
    auto segment = tool_path->begin();

    while (segment != tool_path->end())
    {
      double length;
      std::vector<double> distances;
      std::tie(length, distances) = computeLength(*segment);
      if (length < minimum_length_)
        segment = tool_path->erase(segment);
      else
        ++segment;
    }

    if (tool_path->empty())
      tool_path = tool_paths.erase(tool_path);
    else
      ++tool_path;
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::MinimumSegmentLengthToolPathModifier>::encode(
    const noether::MinimumSegmentLengthToolPathModifier& val)
{
  Node node;
  node["minimum_length"] = val.minimum_length_;
  return node;
}

bool convert<noether::MinimumSegmentLengthToolPathModifier>::decode(const Node& node,
                                                                    noether::MinimumSegmentLengthToolPathModifier& val)
{
  val.minimum_length_ = getMember<double>(node, "minimum_length");
  return true;
}
/** @endcond */
}  // namespace YAML
