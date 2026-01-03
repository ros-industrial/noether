#include <noether_tpp/tool_path_modifiers/uniform_spacing_modifier.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/splines.h>

namespace noether
{
UniformSpacingModifier::UniformSpacingModifier(const double point_spacing,
                                               const long spline_degree,
                                               const bool include_endpoints)
  : ToolPathModifier()
  , point_spacing_(point_spacing)
  , spline_degree_(spline_degree)
  , include_endpoints_(include_endpoints)
{
}

noether::ToolPathSegment UniformSpacingModifier::resample(const noether::ToolPathSegment& segment) const
{
  if (segment.size() < spline_degree_ + 2)
    throw std::runtime_error("Spline cannot be fit to tool path segment with less than degree+2 waypoints");

  // Extract the spline points
  Eigen::Matrix3Xd points(3, segment.size());
  for (std::size_t i = 0; i < segment.size(); ++i)
    points.col(i) = segment[i].translation();

  // Interpolate the spline
  const Eigen::Spline3d spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(points, spline_degree_);

  // Compute the length of the spline and estimate the number of new samples
  std::vector<double> u = computeEquidistantKnots(spline, point_spacing_, include_endpoints_);

  // Allocate the output segment
  noether::ToolPathSegment output;
  output.reserve(u.size());

  // Use the first waypoint z-axis as the reference normal
  Eigen::Vector3d ref_z = segment.front().matrix().col(2).head<3>();

  for (std::size_t i = 0; i < u.size(); ++i)
  {
    Eigen::Isometry3d pose = sample(spline, u[i], ref_z);
    output.push_back(pose);

    // Update the reference z-axis for the next waypoint
    ref_z = pose.matrix().col(2).head<3>();
  }

  return output;
}

ToolPaths UniformSpacingModifier::modify(ToolPaths tool_paths) const
{
  ToolPaths output;
  output.reserve(tool_paths.size());

  for (const ToolPath& tool_path : tool_paths)
  {
    ToolPath tp;
    tp.reserve(tool_path.size());

    for (const ToolPathSegment& segment : tool_path)
      tp.push_back(resample(segment));

    output.push_back(tp);
  }

  return output;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::UniformSpacingModifier>::encode(const noether::UniformSpacingModifier& val)
{
  Node node;
  node["point_spacing"] = val.point_spacing_;
  node["spline_degree"] = val.spline_degree_;
  node["include_endpoints"] = val.include_endpoints_;
  return node;
}

bool convert<noether::UniformSpacingModifier>::decode(const Node& node, noether::UniformSpacingModifier& val)
{
  val.point_spacing_ = getMember<double>(node, "point_spacing");
  val.spline_degree_ = getMember<long>(node, "spline_degree");
  val.include_endpoints_ = getMember<bool>(node, "include_endpoints");
  return true;
}
/** @endcond */

}  // namespace YAML
