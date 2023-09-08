#include <noether_tpp/tool_path_modifiers/linear_approach_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
  LinearApproachModifier(Eigen::Vector3d offset, std::size_t n_points)
    : offset_(offset), n_points_(n_points)
  {
  }

  LinearApproachModifier(Eigen::Vector3d offset, Axis axis, std::size_t n_points)
    : n_points_(n_points)
  {
    switch(axis)
    {
      case Axis::X:
        offset_ = Eigen::Vector3d::UnitX() * offset;
      case Axis::Y:
        offset_ = Eigen::Vector3d::UnitY() * offset;
      case Axis::Z:
        offset_ = Eigen::Vector3d::UnitZ() * offset;
      default:
        throw std::runtime_error("Invalid axis enumeration");
    }
  }

ToolPaths LinearApproachModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      Eigen::Isometry3d offset_point = segment.front() * Eigen::Translation3d(0.0, 0.0, offset_height_);
      ToolPathSegment new_segment;


      for (int i = 0; i < n_points_; i++)
      {
        Eigen::Isometry3d pt;
        pt = offset_point * Eigen::Translation3d(0, 0, -(offset_/(n_points_)*i));
        pt.linear() = segment.front().linear();
        new_segment.push_back(pt);
      }

    segment.insert(segment.begin(), new_segment.begin(), new_segment.end());
    }
  }

  return tool_paths;
}

}  // namespace noether
