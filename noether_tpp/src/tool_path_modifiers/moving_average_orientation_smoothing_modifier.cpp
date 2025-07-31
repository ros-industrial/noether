#include <noether_tpp/tool_path_modifiers/moving_average_orientation_smoothing_modifier.h>
#include <noether_tpp/serialization.h>

namespace noether
{
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

  if (q.coeffs().array().isNaN().any())
    throw std::runtime_error("Orientation contains NaN values");

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
    for (int i = start_idx; i < stop_idx + 1; ++i)
    {
      q.push_back(Eigen::Quaterniond(input_poses.at(i).linear()));
    }

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
        tps[i].linear() = getMovingAverageQuaternion(tps, static_cast<int>(i), window_size_).toRotationMatrix();
      }
    }
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
Node convert<noether::MovingAverageOrientationSmoothingModifier>::encode(const T& val)
{
  Node node;
  node["window_size"] = val.window_size_;
  return node;
}

bool convert<noether::MovingAverageOrientationSmoothingModifier>::decode(const Node& node, T& val)
{
  val.window_size_ = static_cast<std::size_t>(getMember<double>(node, "window_size"));
  return true;
}

}  // namespace YAML
