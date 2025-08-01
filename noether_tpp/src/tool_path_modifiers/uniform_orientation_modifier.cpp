#include <noether_tpp/tool_path_modifiers/uniform_orientation_modifier.h>
#include <noether_tpp/serialization.h>

namespace noether
{
ToolPaths UniformOrientationModifier::modify(ToolPaths tool_paths) const
{
  // Get the reference direction of travel from the first two waypoints
  const Eigen::Isometry3d& w0 = tool_paths.at(0).at(0).at(0);
  const Eigen::Isometry3d& w1 = tool_paths.at(0).at(0).at(1);
  const Eigen::Vector3d reference_x_dir = (w1.translation() - w0.translation()).normalized();

  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      for (Eigen::Isometry3d& waypoint : segment)
      {
        // Rotate the waypoint 180 degrees if its x-axis does not align with the reference direction
        double dp = reference_x_dir.dot(waypoint.matrix().col(0).head<3>());
        if (dp < 0.0)
        {
          waypoint = waypoint * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
        }
      }
    }
  }

  return tool_paths;
};

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::UniformOrientationModifier>::encode(const noether::UniformOrientationModifier& val) { return {}; }

bool convert<noether::UniformOrientationModifier>::decode(const Node& node, noether::UniformOrientationModifier& val)
{
  return true;
}
/** @endcond */

}  // namespace YAML
