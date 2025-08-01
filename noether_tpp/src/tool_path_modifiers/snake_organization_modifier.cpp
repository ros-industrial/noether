#include <noether_tpp/tool_path_modifiers/snake_organization_modifier.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/serialization.h>

#include <numeric>

namespace noether
{
ToolPaths SnakeOrganizationModifier::modify(ToolPaths tool_paths) const
{
  // Re-sort the tool paths at odd indices
  for (std::size_t i = 1; i < tool_paths.size(); i += 2)
  {
    ToolPath& tool_path = tool_paths.at(i);

    // Sort waypoints in each tool path segment by descending x-axis value (i.e. right to left)
    for (ToolPathSegment& segment : tool_path)
    {
      std::reverse(segment.begin(), segment.end());
    }

    std::reverse(tool_path.begin(), tool_path.end());
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::SnakeOrganizationModifier>::encode(const noether::SnakeOrganizationModifier& val) { return {}; }

bool convert<noether::SnakeOrganizationModifier>::decode(const Node& node, noether::SnakeOrganizationModifier& val)
{
  return true;
}
/** @endcond */

}  // namespace YAML
