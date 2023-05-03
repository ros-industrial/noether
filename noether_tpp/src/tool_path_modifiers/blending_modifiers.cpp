#include <noether_tpp/tool_path_modifiers/blending_modifiers.h>

namespace noether
{
AngledOrientationModifier::AngledOrientationModifier(double angle_offset, double tool_radius, bool flip_sign)
  : angle_offset_(angle_offset), tool_radius_(tool_radius), flip_sign_(flip_sign)
{
}

ToolPaths AngledOrientationModifier::modify(ToolPaths tool_paths) const
{
  double sign = 1.0;

  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      for (Eigen::Isometry3d& waypoint : segment)
      {
        waypoint.rotate(Eigen::AngleAxisd(sign * -angle_offset_, Eigen::Vector3d::UnitY())).translate(Eigen::Vector3d(sign * tool_radius_,0,0));
      }
    }

    if(flip_sign_)
      sign *= -1.0;
  }

  return tool_paths;
}

LeadInModifier::LeadInModifier(double arc_angle, double arc_radius, double n_points)
  : arc_angle_(arc_angle), arc_radius_(arc_radius), n_points_(n_points)
{
}

ToolPaths LeadInModifier::modify(ToolPaths tool_paths) const
{
  double delta_theta = arc_angle_ / (n_points_ - 1);
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      Eigen::Isometry3d radius_center = segment.front() * Eigen::Translation3d(0.0, 0.0, arc_radius_);
      ToolPathSegment new_segment;

      for(int i = 0; i < n_points_; i++)
      {        Eigen::Isometry3d pt = radius_center * Eigen::AngleAxisd(delta_theta * (i + 1), Eigen::Vector3d::UnitY()) * Eigen::Translation3d(0, 0,-arc_radius_);
        pt.linear() = segment.front().linear();
        new_segment.push_back(pt);
      }

      segment.insert(segment.begin(), new_segment.rbegin(), new_segment.rend());
    }
  }

  return tool_paths;
}

  LeadOutModifier::LeadOutModifier(double arc_angle, double arc_radius, double n_points)
    : arc_angle_(arc_angle), arc_radius_(arc_radius), n_points_(n_points)
  {
  }

  ToolPaths LeadOutModifier::modify(ToolPaths tool_paths) const
  {
    double delta_theta = arc_angle_ / (n_points_ - 1);

    for (ToolPath& tool_path : tool_paths)
    {
      for (ToolPathSegment& segment : tool_path)
      {
        Eigen::Isometry3d radius_center = segment.back() * Eigen::Translation3d(0.0, 0.0, arc_radius_);
        ToolPathSegment new_segment;

        for(int i = 0; i < n_points_; i++)
        {
          Eigen::Isometry3d pt = radius_center * Eigen::AngleAxisd(-delta_theta * (i + 1), Eigen::Vector3d::UnitY()) * Eigen::Translation3d(0, 0, -arc_radius_);
          pt.linear() = segment.back().linear();
          new_segment.insert(new_segment.begin(), pt);
        }

        segment.insert(segment.end(), new_segment.begin(), new_segment.end());
      }
    }

    return tool_paths;
  }

} // namespace noether
