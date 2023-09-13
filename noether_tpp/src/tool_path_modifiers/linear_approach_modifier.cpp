#include <noether_tpp/tool_path_modifiers/linear_approach_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
  LinearApproachModifier::LinearApproachModifier(Eigen::Vector3d offset, std::size_t n_points)
    : offset_(offset), n_points_(n_points)
  {
  }

ToolPaths LinearApproachModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      Eigen::Isometry3d offset_point = segment.front() * Eigen::Translation3d(offset_);
      ToolPathSegment new_segment;


      for (int i = 0; i < n_points_; i++)
      {
        Eigen::Isometry3d pt;
        pt = offset_point * Eigen::Translation3d(-(offset_/(n_points_)*i));
        pt.linear() = segment.front().linear();
        new_segment.push_back(pt);
      }

    segment.insert(segment.begin(), new_segment.begin(), new_segment.end());
    }
  }

  return tool_paths;
}

}  // namespace noether
