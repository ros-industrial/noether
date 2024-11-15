#include <noether_tpp/tool_path_modifiers/biased_tool_drag_orientation_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
BiasedToolDragOrientationToolPathModifier::BiasedToolDragOrientationToolPathModifier(double angle_offset,
                                                                                     double tool_radius)
  : angle_offset_(angle_offset), tool_radius_(tool_radius)
{
}

ToolPaths BiasedToolDragOrientationToolPathModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    Eigen::Vector3d dir = estimateToolPathDirection(tool_path);
    const Eigen::Isometry3d& first = tool_path.at(0).at(0);
    const Eigen::Vector3d& y = first.matrix().col(1).head<3>();
    const Eigen::Vector3d& z = first.matrix().col(2).head<3>();
    // Sign is determined by whether the first point's y-axis is aligned with the nominal path y-axis (i.e., the cross
    // of the waypoint's z-axis with the nominal path direction)
    double sign = z.cross(dir).dot(y) > 0.0 ? 1.0 : -1.0;
    for (ToolPathSegment& segment : tool_path)
    {
      for (Eigen::Isometry3d& waypoint : segment)
      {
        waypoint.rotate(Eigen::AngleAxisd(sign * -angle_offset_, Eigen::Vector3d::UnitY()))
            .translate(Eigen::Vector3d(sign * tool_radius_, 0, 0));
      }
    }
  }

  return tool_paths;
}

}  // namespace noether
