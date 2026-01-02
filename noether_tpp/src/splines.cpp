#include <noether_tpp/splines.h>

namespace noether
{
/**
 * @details Implments Simpson's composite 3/8 rule for approximate numerical integration
 * @sa https://en.wikipedia.org/wiki/Simpson%27s_rule#Composite_Simpson's_3/8_rule
 * @param f Function to integrate (e.g., spline derivative)
 * @param a Start location (e.g., spline start parameter)
 * @param b End location (e.g., spline end parameter)
 * @param n Number of subdivisions (must be a multiple of 3)
 * @return
 */
double
simpsonComposite38(const std::function<Eigen::ArrayXd(double)>& f, const double a, const double b, const int n = 3)
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

Eigen::Isometry3d sample(const Eigen::Spline3d& spline, const double t, const Eigen::Vector3d& ref_z)
{
  // Evaluate the spline and its first derivative at the given spline parameter
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

std::vector<double> computeSpanLengths(const Eigen::Spline3d& spline)
{
  // Create a function for evaluating the first derivative of the spline (i.e., its velocity)
  // This function will be integrated using Simpson's rule to compute the distance along the spline (i.e., integral of
  // spline velocity)
  std::function<Eigen::Vector3d(const double)> f = [&spline](double u) { return spline.derivatives(u, 1).col(1); };

  // Iterate over the polynomial defining each segment of the spline
  std::vector<double> span_lengths;
  span_lengths.reserve(spline.knots().size() - 1);
  for (Eigen::Index i = 0; i < spline.knots().size() - 1; ++i)
  {
    double a = spline.knots()(i);
    double b = spline.knots()(i + 1);
    span_lengths.push_back(simpsonComposite38(f, a, b));
  }
  return span_lengths;
}

std::vector<double> computeKnotDistances(const Eigen::Spline3d& spline)
{
  const std::vector<double> span_lengths = computeSpanLengths(spline);

  std::vector<double> knot_distances;
  knot_distances.reserve(span_lengths.size() + 1);

  // Add a knot distance for the start of the spline at distance 0.0
  knot_distances.push_back(0.0);

  // Compute the cumulative sum of the span lengths and add to the end of the knot distances
  std::partial_sum(span_lengths.begin(), span_lengths.end(), std::back_inserter(knot_distances));

  return knot_distances;
}

double computeLength(const Eigen::Spline3d& spline)
{
  const std::vector<double> span_lengths = computeSpanLengths(spline);
  return std::accumulate(span_lengths.begin(), span_lengths.end(), 0.0);
}

/**
 * @details Returns the 1) distance and 2) value of the closest knot to the desired distance value and 3) a knot offset
 * value from the closest knot that approximates the location of the desired distance. The estimated spline parameter at
 * the input distance is the knot value plus the offset.
 * @param spline
 * @param dist Desired distance (m) along the spline
 * @return Tuple of [Closest knot distance, closest knot value, approximate knot offset to desired distance]
 */
std::tuple<double, double, double> estimateSplineParameterAtDistance(const Eigen::Spline3d& spline, const double dist)
{
  // Compute the knot distances of the spline
  const std::vector<double> knot_distances = computeKnotDistances(spline);

  // Check if the distance corresponds to the end of one of the knots
  {
    auto d_it = std::find_if(knot_distances.begin(), knot_distances.end(), [dist](const double v) {
      return std::abs(v - dist) < std::numeric_limits<double>::epsilon();
    });
    if (d_it != knot_distances.end())
    {
      const auto knot_idx = std::distance(knot_distances.begin(), d_it);
      return std::make_tuple(*d_it, spline.knots()(knot_idx), 0.0);
    }
  }

  // Otherwise, find the span in which the desired distance falls
  // std::lower_bound returns an iterator to the first value that is not ordered before dist -> i.e., the end distance
  // of the knot span
  auto d_end = std::lower_bound(knot_distances.begin(), knot_distances.end(), dist);

  // Handle the case for extrapolation before the start of the spline
  if (d_end == knot_distances.begin())
    return std::make_tuple(0.0, 0.0, std::copysign(0.1, dist));

  // Handle the case for extrapolation after the end of the spline
  if (d_end == knot_distances.end())
    return std::make_tuple(knot_distances.back(), 1.0, std::copysign(0.1, dist));

  // Handle the nominal interpolation case
  const auto idx_end = std::distance(knot_distances.begin(), d_end);
  auto d_begin = d_end - 1;
  const double t_end = spline.knots()(idx_end);
  const double t_begin = spline.knots()(idx_end - 1);
  const double dt = ((dist - *d_begin) / (*d_end - *d_begin)) * (t_end - t_begin);

  return std::make_tuple(*d_begin, t_begin, dt);
}

double computeSplineParameterAtDistance(const Eigen::Spline3d& spline, const double dist, const double tol)
{
  double s0;  // Spline distance at closest knot
  double t;   // Closest knot
  double dt;  // Estimated knot offset to desired distance
  std::tie(s0, t, dt) = estimateSplineParameterAtDistance(spline, dist);

  // Create a function for evaluating the first derivative of the spline (i.e., its velocity)
  // This function will be integrated using Simpson's rule to compute the distance along the spline (i.e., integral of
  // spline velocity)
  std::function<Eigen::Array3d(const double)> f = [&spline](double u) { return spline.derivatives(u, 1).col(1); };

  // Iteratively refine dt until the computed distance equals the desired distance (within
  // the specified tolerance)
  double s = s0;
  double dt_prev = dt;
  while (std::abs(s - dist) > tol)
  {
    // Compute the offset distance using Simpson's 3/8 rule to integrate the spline derivative
    double ds = std::copysign(simpsonComposite38(f, t, t + dt), dist);
    s = s0 + ds;

    // Scale the dt estimate based on the ratio of desired remaining distance to computed distance
    dt_prev = dt;
    dt *= (dist - s0) / ds;
  }

  return t + dt_prev;
}

std::vector<double>
computeEquidistantKnots(const Eigen::Spline3d& spline, const double spacing, bool include_endpoints, const double tol)
{
  // Allocate the output vector of spline parameters
  std::vector<double> t;

  // Compute the computeLength of the spline
  double l = computeLength(spline);

  // Determine the start distance the first point
  // Split the remaining distance between the start and end points
  double s;
  double remainder = std::fmod(l, spacing);
  if (remainder < (spacing / 2.0))
  {
    s = spacing / 2.0 + remainder / 2.0;
  }
  else
  {
    s = remainder / 2.0;
  }

  // Iteratively add spline parameter values until the total spline length is reached
  while (s < l)
  {
    t.push_back(computeSplineParameterAtDistance(spline, s, tol));
    s += spacing;
  }

  if (include_endpoints)
  {
    t.insert(t.begin(), 0.0);
    t.push_back(1.0);
  }

  return t;
}

}  // namespace noether
