#include <noether_tpp/tool_path_modifiers/linear_departure_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
LinearDepartureModifier::LinearDepartureModifier(Eigen::Vector3d offset, double n_points)
  : offset_(offset), n_points_(n_points)
{
}

ToolPaths LinearDepartureModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      Eigen::Isometry3d offset_point = segment.back() * Eigen::Translation3d(offset_);
      ToolPathSegment new_segment;

      for (int i = 0; i <= n_points_; i++)
      {
        Eigen::Isometry3d pt;
        pt = offset_point * Eigen::Translation3d(-offset_ + (offset_/(n_points_))*i);
        pt.linear() = segment.back().linear();
        new_segment.push_back(pt);
      }

      segment.insert(segment.end(), new_segment.rbegin(), new_segment.rend());
    }
  }

  return tool_paths;
}

}  // namespace noether

