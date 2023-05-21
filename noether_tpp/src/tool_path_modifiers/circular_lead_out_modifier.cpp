#include <noether_tpp/tool_path_modifiers/circular_lead_out_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
CircularLeadOutModifier::CircularLeadOutModifier(double arc_angle, double arc_radius, double n_points)
  : arc_angle_(arc_angle), arc_radius_(arc_radius), n_points_(n_points)
{
}

ToolPaths CircularLeadOutModifier::modify(ToolPaths tool_paths) const
{
  double delta_theta = arc_angle_ / (n_points_ - 1);
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
      Eigen::Isometry3d radius_center = segment.back() * Eigen::Translation3d(0.0, 0.0, arc_radius_);
      ToolPathSegment new_segment;

      for (int i = 0; i < n_points_; i++)
      {
        Eigen::Isometry3d pt = radius_center *
                               Eigen::AngleAxisd(sign * -delta_theta * (i + 1), Eigen::Vector3d::UnitY()) *
                               Eigen::Translation3d(0, 0, -arc_radius_);
        pt.linear() = segment.back().linear();
        new_segment.insert(new_segment.begin(), pt);
      }

      segment.insert(segment.end(), new_segment.rbegin(), new_segment.rend());
    }
  }

  return tool_paths;
}

}  // namespace noether
