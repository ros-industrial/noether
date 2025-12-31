#include <noether_tpp/tool_path_modifiers/uniform_spacing_linear_modifier.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/serialization.h>

#include <unsupported/Eigen/Splines>

/** @brief Typedef for a 3-dimensional spline of specified degree */
using Spline = Eigen::Spline<double, 3, Eigen::Dynamic>;

using functor = std::function<Eigen::Array3d(double)>;

/**
 * @details Implments Simpson's composite 3/8 rule for approximate numerical integration
 * @sa https://en.wikipedia.org/wiki/Simpson%27s_rule#Composite_Simpson's_3/8_rule
 * @param f Function to integrate (e.g., spline derivative)
 * @param a Start location (e.g., spline start "time")
 * @param b End location (e.g., spline end "time")
 * @param n Number of subdivisions (must be a multiple of 3)
 * @return
 */
static double simpsonComposite38(const functor& f, const double a, const double b, const int n = 3)
{
  if ((n % 3) != 0)
    throw std::runtime_error("Number of subdivisions for Simpson's 3/8 rule must be a multiple of 3");

  const double h = (b - a) / n;
  const Eigen::ArrayXd x = Eigen::ArrayXd::LinSpaced(3 * n + 1, a, b);

  Eigen::Array3d dist = Eigen::Array3d::Zero();
  for (std::size_t i = 1; i <= n / 3; ++i)
  {
    std::size_t idx_1 = 3 * i - 3;
    std::size_t idx_2 = 3 * i - 2;
    std::size_t idx_3 = 3 * i - 1;
    std::size_t idx_4 = 3 * i;
    dist += f(x(idx_1)) + 3.0 * f(x(idx_2)) + 3.0 * f(x(idx_3)) + f(x(idx_4));
  }

  dist *= 3.0 * h / 8.0;

  return dist.matrix().norm();
}

/**
 * @brief Computes the length of a spline using Simpson's composite 3/8 rule
 * @param spline
 * @return
 */
double length(const Spline& spline)
{
  // Create a function for evaluating the first derivative of the spline (i.e., its velocity)
  // This function will be integrated using Simpson's rule to compute the distance along the spline (i.e., integral of
  // spline velocity)
  functor f = [&spline](double u) { return spline.derivatives(u, 1).col(1); };

  // Iterate over the polynomial defining each segment of the spline
  double l = 0.0;
  for (Eigen::Index i = 0; i < spline.knots().size() - 1; ++i)
  {
    double a = spline.knots()(i);
    double b = spline.knots()(i + 1);
    l += simpsonComposite38(f, a, b);
  }
  return l;
}

noether::ToolPathSegment sample(const noether::ToolPathSegment& segment,
                                const long spline_degree,
                                const double point_spacing)
{
  // Extract the spline points
  Eigen::Matrix3Xd points(3, segment.size());
  for (std::size_t i = 0; i < segment.size(); ++i)
    points.col(i) = segment[i].translation();

  // Interpolate the spline
  const Spline spline = Eigen::SplineFitting<Spline>::Interpolate(points, spline_degree);

  // Compute the length of the spline and estimate the number of new samples
  const double l = length(spline);
  auto n = static_cast<Eigen::Index>(std::ceil(l / point_spacing));
  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(n, 0.0, 1.0);

  // Allocate the output segment
  noether::ToolPathSegment output;
  output.reserve(n);

  // Use the first waypoint z-axis as the reference normal
  Eigen::Vector3d ref_z = segment.front().matrix().col(2).head<3>();

  for (Eigen::Index i = 0; i < n; ++i)
  {
    // Evaluate the spline and its first derivative at the given time parameter
    const Eigen::MatrixXd s = spline.derivatives(t(i), 1);

    Eigen::Isometry3d pose;

    // Spline point
    pose.translation() = s.col(0);

    // Spline first derivative; points along the spline
    const Eigen::Vector3d x = s.col(1);
    const Eigen::Vector3d y = ref_z.cross(x);
    const Eigen::Vector3d z = x.cross(y);

    pose.matrix().col(0).head<3>() = x.normalized();
    pose.matrix().col(1).head<3>() = y.normalized();
    pose.matrix().col(2).head<3>() = z.normalized();

    output.push_back(pose);

    // Update the reference z-axis for the next waypoint
    ref_z = z.normalized();
  }

  return output;
}

namespace noether
{
UniformSpacingLinearModifier::UniformSpacingLinearModifier(const double point_spacing, const long spline_degree)
  : ToolPathModifier(), point_spacing_(point_spacing), spline_degree_(spline_degree)
{
}

ToolPaths UniformSpacingLinearModifier::modify(ToolPaths tool_paths) const
{
  ToolPaths output;
  output.reserve(tool_paths.size());

  for (const ToolPath& tool_path : tool_paths)
  {
    ToolPath tp;
    tp.reserve(tool_path.size());

    for (const ToolPathSegment& segment : tool_path)
    {
      if (segment.size() < spline_degree_ + 2)
        throw std::runtime_error("Spline cannot be fit to tool path segment with less than degree+2 waypoints");

      tp.push_back(sample(segment, spline_degree_, point_spacing_));
    }

    output.push_back(tp);
  }

  return output;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::UniformSpacingLinearModifier>::encode(const noether::UniformSpacingLinearModifier& val)
{
  Node node;
  node["point_spacing"] = val.point_spacing_;
  node["spline_degree"] = val.spline_degree_;
  return node;
}

bool convert<noether::UniformSpacingLinearModifier>::decode(const Node& node,
                                                            noether::UniformSpacingLinearModifier& val)
{
  val.point_spacing_ = getMember<double>(node, "point_spacing");
  if (node["spline_degree"])
    val.spline_degree_ = getMember<long>(node, "spline_degree");
  else
    val.spline_degree_ = 1;
  return true;
}
/** @endcond */

}  // namespace YAML
