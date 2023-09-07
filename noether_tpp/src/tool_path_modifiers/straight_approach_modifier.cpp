#include <noether_tpp/tool_path_modifiers/straight_approach_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
StraightApproachModifier::StraightApproachModifier(double offset_height, double n_points)
  : offset_height_(offset_height), n_points_(n_points)
{
}

ToolPaths StraightApproachModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    Eigen::Vector3d dir = estimateToolPathDirection(tool_path);
    const Eigen::Isometry3d& first = tool_path.at(0).at(0);
    const Eigen::Vector3d& y = first.matrix().col(1).head<3>();
    const Eigen::Vector3d& z = first.matrix().col(2).head<3>();
    for (ToolPathSegment& segment : tool_path)
    {
      Eigen::Isometry3d radius_center = segment.front() * Eigen::Translation3d(0.0, 0.0, offset_height_);
      ToolPathSegment new_segment;

      for (int i = 0; i < n_points_; i++)
      {
        Eigen::Isometry3d pt;
        pt = radius_center * Eigen::Translation3d(0, 0, -offset_height_ + (offset_height_/(n_points_))*i);
        pt.linear() = segment.front().linear();
        new_segment.push_back(pt);
      }

      segment.insert(segment.begin(), new_segment.rbegin(), new_segment.rend());
    }
  }

  return tool_paths;
}

}  // namespace noether
