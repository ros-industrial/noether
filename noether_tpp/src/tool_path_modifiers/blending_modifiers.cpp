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

} // namespace noether
