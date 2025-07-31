#include <noether_tpp/tool_path_modifiers/linear_approach_modifier.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/serialization.h>

namespace noether
{
Eigen::Vector3d LinearApproachModifier::toVector(double offset, Axis axis)
{
  switch (axis)
  {
    case Axis::X:
      return Eigen::Vector3d::UnitX() * offset;
    case Axis::Y:
      return Eigen::Vector3d::UnitY() * offset;
    case Axis::Z:
      return Eigen::Vector3d::UnitZ() * offset;
    default:
      throw std::runtime_error("Invalid axis value");
  }
}

LinearApproachModifier::LinearApproachModifier(Eigen::Vector3d offset, std::size_t n_points)
  : offset_(offset), n_points_(n_points)
{
}

LinearApproachModifier::LinearApproachModifier(double offset, Axis axis, std::size_t n_points)
  : offset_(toVector(offset, axis)), n_points_(n_points)
{
}

ToolPaths LinearApproachModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      ToolPathSegment new_segment;
      for (int i = 0; i < n_points_; i++)
      {
        Eigen::Isometry3d pt =
            segment.front() *
            Eigen::Translation3d(offset_ * (static_cast<double>(i + 1) / static_cast<double>(n_points_)));
        pt.linear() = segment.front().linear();
        new_segment.push_back(pt);
      }

      segment.insert(segment.begin(), new_segment.rbegin(), new_segment.rend());
    }
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
Node convert<noether::LinearApproachModifier>::encode(const T& val)
{
  Node node;
  node["offset"] = val.offset_;
  node["n_points"] = val.n_points_;
  return node;
}

bool convert<noether::LinearApproachModifier>::decode(const Node& node, T& val)
{
  val.offset_ = getMember<Eigen::Vector3d>(node, "offset");
  val.n_points_ = getMember<int>(node, "n_points");
  return true;
}

}  // namespace YAML
