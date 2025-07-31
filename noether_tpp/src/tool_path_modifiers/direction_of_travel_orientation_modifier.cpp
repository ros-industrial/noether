#include <noether_tpp/tool_path_modifiers/direction_of_travel_orientation_modifier.h>
#include <noether_tpp/serialization.h>

namespace noether
{
ToolPaths DirectionOfTravelOrientationModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      for (std::size_t i = 0; i < segment.size(); ++i)
      {
        // Calculate the reference direction based on the waypoint adjacent to the current waypoint
        Eigen::Vector3d reference_x_dir;
        if (i < segment.size() - 1)
        {
          const Eigen::Isometry3d& first = segment[i];
          const Eigen::Isometry3d& second = segment[i + 1];
          reference_x_dir = (second.translation() - first.translation()).normalized();
        }
        else
        {
          const Eigen::Isometry3d& first = segment[i - 1];
          const Eigen::Isometry3d& second = segment[i];
          reference_x_dir = (second.translation() - first.translation()).normalized();
        }

        Eigen::Isometry3d& waypoint = segment[i];

        // Keep the z-axis from the original waypoint
        const Eigen::Vector3d z_axis = waypoint.matrix().col(2).head<3>();

        // Compute the new y-axis from the original z-axis and the reference direction
        const Eigen::Vector3d y_axis = z_axis.cross(reference_x_dir);

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
Node convert<noether::DirectionOfTravelOrientationModifier>::encode(
    const noether::DirectionOfTravelOrientationModifier& val)
{
  return {};
}

bool convert<noether::DirectionOfTravelOrientationModifier>::decode(const Node& node,
                                                                    noether::DirectionOfTravelOrientationModifier& val)
{
  return true;
}
/** @endcond */

}  // namespace YAML
