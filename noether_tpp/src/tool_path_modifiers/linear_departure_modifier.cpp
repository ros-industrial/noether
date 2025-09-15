#include <noether_tpp/tool_path_modifiers/linear_departure_modifier.h>
#include <noether_tpp/serialization.h>

namespace noether
{
ToolPaths LinearDepartureModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      ToolPathSegment new_segment;

      for (int i = 0; i < n_points_; i++)
      {
        Eigen::Isometry3d pt =
            segment.back() *
            Eigen::Translation3d(offset_ * (static_cast<double>(i + 1) / static_cast<double>(n_points_)));
        pt.linear() = segment.back().linear();
        new_segment.push_back(pt);
      }

      segment.insert(segment.end(), new_segment.begin(), new_segment.end());
    }
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::LinearDepartureModifier>::encode(const noether::LinearDepartureModifier& val)
{
  return convert<noether::LinearApproachModifier>::encode(val);
}

bool convert<noether::LinearDepartureModifier>::decode(const Node& node, noether::LinearDepartureModifier& val)
{
  return convert<noether::LinearApproachModifier>::decode(node, val);
}
/** @endcond */

}  // namespace YAML
