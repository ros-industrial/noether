#include <noether_tpp/tool_path_modifiers/organization_modifiers.h>

namespace noether
{
ToolPaths RasterOrganizationModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    // Sort waypoints in each tool path segment by ascending x-axis value (i.e. left to right)
    for (ToolPathSegment& segment : tool_path)
    {
      std::sort(segment.begin(), segment.end(), [](const ToolPathWaypoint& a, const ToolPathWaypoint& b) {
        return a.translation().x() < b.translation().x();
      });
    }

    // Sort the tool path segments by ascending x-axis value of the first waypoint in each tool path segment
    std::sort(tool_path.begin(), tool_path.end(), [](const ToolPathSegment& a, const ToolPathSegment& b) {
      return a.at(0).translation().x() < b.at(0).translation().x();
    });
  }

  // Sort the tool paths by ascending y-axis value of the first waypoint in the first tool path segment of each tool
  // path
  std::sort(tool_paths.begin(), tool_paths.end(), [](const ToolPath& a, const ToolPath& b) {
    return a.at(0).at(0).translation().y() < b.at(0).at(0).translation().y();
  });

  return tool_paths;
}

ToolPaths SnakeOrganizationModifier::modify(ToolPaths tool_paths) const
{
  // Create a raster
  RasterOrganizationModifier raster;
  tool_paths = raster.modify(tool_paths);

  // Re-sort the tool paths at odd indices
  for (std::size_t i = 1; i < tool_paths.size(); i += 2)
  {
    ToolPath& tool_path = tool_paths.at(i);

    // Sort waypoints in each tool path segment by descending x-axis value (i.e. right to left)
    for (ToolPathSegment& segment : tool_path)
    {
      std::sort(segment.begin(), segment.end(), [](const ToolPathWaypoint& a, const ToolPathWaypoint& b) {
        return a.translation().x() > b.translation().x();
      });
    }

    // Sort the tool path segments by descending x-axis value of the first waypoint in each tool path segment
    std::sort(tool_path.begin(), tool_path.end(), [](const ToolPathSegment& a, const ToolPathSegment& b) {
      return a.at(0).translation().x() > b.at(0).translation().x();
    });
  }

  return tool_paths;
}

}  // namespace noether
