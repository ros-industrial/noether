#include <noether_tpp/tool_path_modifiers/radius_of_curvature_extrapolation_modifier.h>
#include <noether_tpp/serialization.h>

#include <unsupported/Eigen/Splines>

/**
 * @details Implments Simpson's composite 3/8 rule for approximate numerical integration
 * @sa https://en.wikipedia.org/wiki/Simpson%27s_rule#Composite_Simpson's_3/8_rule
 * @param f Function to integrate (e.g., spline derivative)
 * @param a Start location (e.g., spline start "time")
 * @param b End location (e.g., spline end "time")
 * @param n Number of subdivisions (must be a multiple of 3)
 * @return
 */
double
simpsonComposite38(const std::function<Eigen::Array3d(double)>& f, const double a, const double b, const int n = 3)
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
 * @details Computes the spline parameter (i.e., time) at a specific extrapolated distance on the spline
 * @param spline
 * @param dist Extrapolation distance (m). Positive values indicate extrapolation beyond the end of the spline; negative
 * values indicate extrapolation before the start of the spline.
 * @param tol Tolerance (m) at which to approximate the extrapolation distance
 * @return
 */
double getTimeAtExtrapolationDistance(const Eigen::Spline3d& spline, const double dist, const double tol = 1.0e-4)
{
  // Determine the initial spline parameter (i.e., time) based on whether the specified distance is positive or negative
  const double t = dist > 0.0 ? 1.0 : 0.0;

  // Create an initial guess for the extrapolation time
  double dt = std::copysign(0.1, dist);

  // Create a function for evaluating the first derivative of the spline (i.e., its velocity)
  // This function will be integrated using Simpson's rule to compute the distance along the spline (i.e., integral of
  // spline velocity)
  std::function<Eigen::Array3d(double)> f = [&spline](double u) { return spline.derivatives(u, 1).col(1); };

  // Iteratively refine dt until the computed extrapolation distance equals the desired extrapolation distance (within
  // the specified tolerance)
  double d = dist;
  do
  {
    // Scale the dt guess based on the ratio of desired distance to computed distance
    dt *= (dist / d);

    // Use Simpson's 3/8 rule to integrate the spline derivative from the initial time to the extrapolation time
    d = std::copysign(simpsonComposite38(f, t, t + dt), dist);
  } while (std::abs(d - dist) > tol);

  return t + dt;
}

/**
 * @details Extrapolates a spline at some specified distance beyond the bounds of the spline
 * @param spline
 * @param dist Extrapolation distance (m). Positive values indicate extrapolation beyond the end of the spline; negative
 * values indicate extrapolation before the start of the spline.
 * @param ref_z Reference Z-axis for defining the orientation of the extrapolated waypoint
 * @return
 */
Eigen::Isometry3d extrapolate(const Eigen::Spline3d& spline, const double dist, const Eigen::Vector3d& ref_z)
{
  double t = getTimeAtExtrapolationDistance(spline, dist);

  // Evaluate the spline and its first derivative at the given time parameter
  const Eigen::MatrixXd s = spline.derivatives(t, 1);

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

  return pose;
}

namespace noether
{
RadiusOfCurvatureExtrapolationToolPathModifier::RadiusOfCurvatureExtrapolationToolPathModifier(
    const double spline_degree,
    const double extrapolation_distance,
    const double normal_offset_distance,
    const bool extrapolate_front,
    const bool extrapolate_back)
  : spline_degree_(spline_degree)
  , extrapolation_distance_(extrapolation_distance)
  , normal_offset_distance_(normal_offset_distance)
  , extrapolate_front_(extrapolate_front)
  , extrapolate_back_(extrapolate_back)
{
}

ToolPaths RadiusOfCurvatureExtrapolationToolPathModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      if (segment.size() < spline_degree_ + 2)
        throw std::runtime_error("Spline cannot be fit to tool path segment with less than degree+2 waypoints");

      // Extract the spline points
      Eigen::Matrix3Xd points(3, segment.size());
      for (std::size_t i = 0; i < segment.size(); ++i)
        points.col(i) = segment[i].translation();

      // Interpolate the spline
      Eigen::Spline3d spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(points, spline_degree_);

      // Create an extrapolation point at the front of the segment
      if (extrapolate_front_)
      {
        const auto& ref_z = segment.front().matrix().col(2).head<3>();
        Eigen::Isometry3d front = extrapolate(spline, -extrapolation_distance_, ref_z);
        front *= Eigen::Translation3d(0.0, 0.0, normal_offset_distance_);
        segment.insert(segment.begin(), front);
      }

      // Create an extrapolation point at the back of the segment
      if (extrapolate_back_)
      {
        const auto& ref_z = segment.back().matrix().col(2).head<3>();
        Eigen::Isometry3d back = extrapolate(spline, extrapolation_distance_, ref_z);
        back *= Eigen::Translation3d(0.0, 0.0, normal_offset_distance_);
        segment.push_back(back);
      }
    }
  }

  return tool_paths;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::RadiusOfCurvatureExtrapolationToolPathModifier>::encode(
    const noether::RadiusOfCurvatureExtrapolationToolPathModifier& val)
{
  Node node;
  node["spline_degree"] = val.spline_degree_;
  node["extrapolation_distance"] = val.extrapolation_distance_;
  node["normal_offset_distance"] = val.normal_offset_distance_;
  node["extrapolate_front"] = val.extrapolate_front_;
  node["extrapolate_back"] = val.extrapolate_back_;
  return {};
}

bool convert<noether::RadiusOfCurvatureExtrapolationToolPathModifier>::decode(
    const Node& node,
    noether::RadiusOfCurvatureExtrapolationToolPathModifier& val)
{
  val.spline_degree_ = YAML::getMember<double>(node, "spline_degree");
  val.extrapolation_distance_ = YAML::getMember<double>(node, "extrapolation_distance");
  val.normal_offset_distance_ = YAML::getMember<double>(node, "normal_offset_distance");
  val.extrapolate_front_ = YAML::getMember<bool>(node, "extrapolate_front");
  val.extrapolate_back_ = YAML::getMember<bool>(node, "extrapolate_back");
  return true;
}
/** @endcond */
}  // namespace YAML
