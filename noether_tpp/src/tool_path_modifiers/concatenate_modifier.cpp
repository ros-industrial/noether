#include <noether_tpp/tool_path_modifiers/concatenate_modifier.h>
#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>
#include <noether_tpp/utils.h>

#include <numeric>

namespace noether
{
ToolPaths ConcatenateModifier::modify(ToolPaths tool_paths) const
{
  // Create a raster
  RasterOrganizationModifier raster;
  tool_paths = raster.modify(tool_paths);
  ToolPath combined_tool_path;
  ToolPaths combined_tool_paths;

  // Loop through the existing toolpaths to create a single combined toolpath
  for (std::size_t i = 0; i < tool_paths.size(); i += 1)
  {
    ToolPath& tool_path = tool_paths.at(i);

    for (ToolPathSegment& segment : tool_path)
    {
      combined_tool_path.push_back(segment);
    }
  }
  combined_tool_paths.push_back(combined_tool_path);

  return combined_tool_paths;
}

}  // namespace noether
