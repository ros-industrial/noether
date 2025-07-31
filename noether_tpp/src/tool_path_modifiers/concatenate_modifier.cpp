#include <noether_tpp/tool_path_modifiers/concatenate_modifier.h>
#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/serialization.h>

namespace noether
{
ToolPaths ConcatenateModifier::modify(ToolPaths tool_paths) const
{
  ToolPath combined_tool_path;

  // Loop through the existing toolpaths to create a single combined toolpath
  for (const ToolPath& path : tool_paths)
    for (const ToolPathSegment& segment : path)
      combined_tool_path.push_back(segment);

  return { combined_tool_path };
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::ConcatenateModifier>::encode(const noether::ConcatenateModifier& val) { return {}; }

bool convert<noether::ConcatenateModifier>::decode(const Node& node, noether::ConcatenateModifier& val) { return true; }
/** @endcond */

}  // namespace YAML
