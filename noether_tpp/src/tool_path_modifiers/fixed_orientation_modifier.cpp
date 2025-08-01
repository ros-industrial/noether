#include <noether_tpp/tool_path_modifiers/fixed_orientation_modifier.h>
#include <noether_tpp/serialization.h>

namespace noether
{
FixedOrientationModifier::FixedOrientationModifier(const Eigen::Vector3d& reference_x_direction)
  : ref_x_dir_(reference_x_direction.normalized())
{
}

ToolPaths FixedOrientationModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      for (Eigen::Isometry3d& waypoint : segment)
      {
        // Keep the z-axis from the original waypoint
        const Eigen::Vector3d z_axis = waypoint.matrix().col(2).head<3>();

        // Compute the new y-axis from the original z-axis and the reference direction
        const Eigen::Vector3d y_axis = z_axis.cross(ref_x_dir_);

        // Compute the new x-axis from the new y-axis and the original z-axis
        const Eigen::Vector3d x_axis = y_axis.cross(z_axis);

        // Update the waypoint orientation
        waypoint.matrix().col(0).head<3>() = x_axis;
        waypoint.matrix().col(1).head<3>() = y_axis;
      }
    }
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::FixedOrientationModifier>::encode(const noether::FixedOrientationModifier& val)
{
  Node node;
  node["ref_x_dir"] = val.ref_x_dir_;
  return node;
}

bool convert<noether::FixedOrientationModifier>::decode(const Node& node, noether::FixedOrientationModifier& val)
{
  val.ref_x_dir_ = getMember<Eigen::Vector3d>(node, "ref_x_dir");
  return true;
}
/** @endcond */

}  // namespace YAML
