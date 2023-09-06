#include <noether_tpp/tool_path_modifiers/straight_approach_modifier.h>.h>
#include <noether_tpp/utils.h>

namespace noether
{
StraightApproachModifier::StraightApproachModifier(double offset_height, double n_points)
  : offset_height_(offset_height), n_points_(n_points)
{
}

ToolPaths StraightApproachModifier::modify(ToolPaths tool_paths) const
{
//  double delta_theta = arc_angle_ / (n_points_ - 1);
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
      Eigen::Isometry3d radius_center = segment.front() * Eigen::Translation3d(0.0, 0.0, offset_height_);
      ToolPathSegment new_segment;

      for (int i = 0; i < n_points_; i++)
      {
        Eigen::Isometry3d pt;
//        if (i == 0 || i==n_points_-1){
          /*Eigen::Isometry3d */pt = radius_center *
//                               Eigen::AngleAxisd(sign * delta_theta * (i + 1), Eigen::Vector3d::UnitY()) *
                               Eigen::Translation3d(0, 0, -offset_height_ + (offset_height_/(n_points_))*i);
//        }
        pt.linear() = segment.front().linear();
        new_segment.push_back(pt);
      }

      segment.insert(segment.begin(), new_segment.rbegin(), new_segment.rend());
    }
  }

  return tool_paths;
}

}  // namespace noether
