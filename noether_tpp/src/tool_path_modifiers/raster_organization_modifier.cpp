#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>
#include <noether_tpp/utils.h>

#include <numeric>

namespace noether
{
ToolPaths RasterOrganizationModifier::modify(ToolPaths tool_paths) const
{
  const Eigen::Vector3d reference_segment_dir = estimateToolPathDirection(tool_paths.at(0));

  for (ToolPath& tool_path : tool_paths)
  {
    // Sort the waypoints within each tool path segment by their distance along the reference direction
    for (ToolPathSegment& segment : tool_path)
    {
      std::sort(segment.begin(),
                segment.end(),
                [&segment, &reference_segment_dir](const ToolPathWaypoint& a, const ToolPathWaypoint& b) {
                  Eigen::Vector3d diff_from_start_b = b.translation() - segment.at(0).translation();
                  Eigen::Vector3d diff_from_start_a = a.translation() - segment.at(0).translation();
                  return diff_from_start_a.dot(reference_segment_dir) < diff_from_start_b.dot(reference_segment_dir);
                });
    }

    // Sort the tool path segments within each tool path by the distance of their first waypoints along the reference
    // direction
    std::sort(tool_path.begin(),
              tool_path.end(),
              [&tool_path, &reference_segment_dir](const ToolPathSegment& a, const ToolPathSegment& b) {
                Eigen::Vector3d diff_from_start_b = b.at(0).translation() - tool_path.at(0).at(0).translation();
                Eigen::Vector3d diff_from_start_a = a.at(0).translation() - tool_path.at(0).at(0).translation();
                return diff_from_start_a.dot(reference_segment_dir) < diff_from_start_b.dot(reference_segment_dir);
              });
  }

  // Sort the tool paths by their distance along a vector that is perpendicular to the reference direction of travel
  const Eigen::Vector3d reference_tool_paths_dir = estimateRasterDirection(tool_paths, reference_segment_dir);
  const Eigen::Isometry3d first_wp = tool_paths.at(0).at(0).at(0);
  std::sort(tool_paths.begin(),
            tool_paths.end(),
            [&tool_paths, &reference_tool_paths_dir, first_wp](const ToolPath& a, const ToolPath& b) {
              Eigen::Vector3d diff_from_start_b = b.at(0).at(0).translation() - first_wp.translation();
              Eigen::Vector3d diff_from_start_a = a.at(0).at(0).translation() - first_wp.translation();
              return diff_from_start_a.dot(reference_tool_paths_dir) < diff_from_start_b.dot(reference_tool_paths_dir);
            });

  return tool_paths;
}

}  // namespace noether
