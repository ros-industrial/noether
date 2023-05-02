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

LeadInModifier::LeadInModifier(double lead_in_angle, double lead_in_arc_radius, double lead_in_num_of_points)
  : lead_in_angle_(lead_in_angle), lead_in_arc_radius_(lead_in_arc_radius), lead_in_num_of_points_(lead_in_num_of_points)
{
}

ToolPaths LeadInModifier::modify(ToolPaths tool_paths) const
{
  double delta_theta = lead_in_angle_ / (lead_in_num_of_points_ - 1);
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      Eigen::Isometry3d radius_center = segment.front() * Eigen::Translation3d(0.0, 0.0, lead_in_arc_radius_);
      Eigen::Isometry3d wp1 = segment.front();
      Eigen::Isometry3d wp2 = segment[1];
      Eigen::Isometry3d wp21 = wp1.inverse() * wp2;
      Eigen::Vector3d t21 = wp21.translation();
      Eigen::Vector3d rotation_vect = Eigen::Vector3d::UnitZ().cross(t21/t21.norm());

      ToolPathSegment new_segment;

      for(int i = 0; i < lead_in_num_of_points_; i++)
      {
        Eigen::Isometry3d pt = radius_center * Eigen::AngleAxisd(lead_in_angle_ - delta_theta * i, rotation_vect) * Eigen::Translation3d(0, 0,-lead_in_arc_radius_);
        pt.linear() = segment.front().linear();
        new_segment.push_back(pt);
      }

      new_segment.insert(new_segment.end(), segment.begin(), segment.end());
      segment = new_segment;
    }
  }

  return tool_paths;
}

  LeadOutModifier::LeadOutModifier(double lead_out_angle, double lead_out_arc_radius, double lead_out_num_of_points)
    : lead_out_angle_(lead_out_angle), lead_out_arc_radius_(lead_out_arc_radius), lead_out_num_of_points_(lead_out_num_of_points)
  {
  }

  ToolPaths LeadOutModifier::modify(ToolPaths tool_paths) const
  {
    double delta_theta = lead_out_angle_ / (lead_out_num_of_points_ - 1);
    for (ToolPath& tool_path : tool_paths)
    {
      for (ToolPathSegment& segment : tool_path)
      {
        Eigen::Isometry3d radius_center = segment.back() * Eigen::Translation3d(0.0, 0.0, -lead_out_arc_radius_);
        Eigen::Isometry3d wp1 = segment.back();
        Eigen::Isometry3d wp2 = segment.end()[-2];
        Eigen::Isometry3d wp21 = wp1.inverse() * wp2;
        Eigen::Vector3d t21 = wp21.translation();
        Eigen::Vector3d rotation_vect = Eigen::Vector3d::UnitZ().cross(t21/t21.norm());

        ToolPathSegment new_segment;

        for(int i = 0; i < lead_out_num_of_points_; i++)
        {
          Eigen::Isometry3d pt = radius_center * Eigen::AngleAxisd(lead_out_angle_ + delta_theta * i, rotation_vect) * Eigen::Translation3d(0, 0, lead_out_arc_radius_);
          pt.linear() = segment.back().linear();
          new_segment.insert(new_segment.begin(), pt);
        }

        new_segment.insert(new_segment.end(), segment.begin(), segment.end());
        segment = new_segment;
      }
    }

    return tool_paths;
  }

} // namespace noether
