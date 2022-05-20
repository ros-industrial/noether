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

static Eigen::Quaterniond computeQuaternionMean(const std::vector<Eigen::Quaterniond>& quaterns)
{
  /* Mean quaternion is found using method described by Markley et al: Quaternion Averaging
   * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
   *
   * M = sum(w_i * q_i * q_i^T)    Eq. 12
   * q_bar = argmax(q^T * M * q)   Eq. 13
   *
   * "The solution of this maximization problem is well known. The average quaternion is
   * the eigenvector of M corresponding to the maximum eigenvalue."
   *
   * In the above equations, w_i is the weight of the ith quaternion.
   * In this case, all quaternions are equally weighted (i.e. w_i = 1)
   */

  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

  for (const Eigen::Quaterniond& q : quaterns)
  {
    M += q.coeffs() * q.coeffs().transpose();
  }

  // Calculate the SVD of the M matrix
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(M, Eigen::ComputeFullU);

  // The eigenvectors are represented by the columns of the U matrix; the eigenvector corresponding to the largest
  // eigenvalue is in row 0
  Eigen::Quaterniond q;
  q.coeffs() << svd.matrixU().col(0);

  assert(std::isnan(q.w()) == false && std::isnan(q.x()) == false && std::isnan(q.y()) == false &&
         std::isnan(q.z()) == false);

  return q;
};

static Eigen::Quaterniond getMovingAverageQuaternion(const ToolPathSegment& input_poses,
                                                     const int pose_index,
                                                     const int n_pts)
{
  Eigen::Quaterniond out(input_poses[static_cast<std::size_t>(pose_index)].linear());

  int n = static_cast<int>(input_poses.size());
  if (pose_index > 0 && pose_index < n - 1)
  {
    int n_each_side = n_pts / 2;
    int start_idx = std::max(pose_index - n_each_side, 0);
    int stop_idx = std::min(pose_index + n_each_side, n - 1);

    // Accumulate all quaternions
    std::vector<Eigen::Quaterniond> q;
    q.reserve(input_poses.size());
    std::transform(input_poses.begin() + start_idx,
                   input_poses.begin() + stop_idx + 1,
                   std::back_inserter(q),
                   [](const Eigen::Isometry3d& p) { return Eigen::Quaterniond(p.linear()); });

    out = computeQuaternionMean(q);
  }

  return out;
}

MovingAverageOrientationSmoothingModifier::MovingAverageOrientationSmoothingModifier(std::size_t window_size)
  : window_size_(window_size)
{
}

ToolPaths MovingAverageOrientationSmoothingModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tp : tool_paths)
  {
    for (ToolPathSegment& tps : tp)
    {
      for (std::size_t i = 0; i < tps.size(); ++i)
      {
        ToolPathWaypoint& w = tps[i];
        w.linear() = getMovingAverageQuaternion(tps, static_cast<int>(i), window_size_).toRotationMatrix();
      }
    }
  }

  return tool_paths;
}

}  // namespace noether
