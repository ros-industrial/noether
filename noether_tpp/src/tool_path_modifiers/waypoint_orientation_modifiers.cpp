#include <noether_tpp/tool_path_modifiers/waypoint_orientation_modifiers.h>

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
