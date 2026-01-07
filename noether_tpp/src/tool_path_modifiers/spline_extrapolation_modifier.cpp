#include <noether_tpp/tool_path_modifiers/spline_extrapolation_modifier.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/splines.h>

namespace noether
{
SplineExtrapolationToolPathModifier::SplineExtrapolationToolPathModifier(const double spline_degree,
                                                                         const double extrapolation_distance_front,
                                                                         const double normal_offset_distance_front,
                                                                         const double extrapolation_distance_back,
                                                                         const double normal_offset_distance_back)
  : spline_degree_(spline_degree)
  , extrapolation_distance_front_(std::abs(extrapolation_distance_front))
  , normal_offset_distance_front_(normal_offset_distance_front)
  , extrapolation_distance_back_(std::abs(extrapolation_distance_back))
  , normal_offset_distance_back_(normal_offset_distance_back)
{
}

ToolPaths SplineExtrapolationToolPathModifier::modify(ToolPaths tool_paths) const
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
      const Eigen::Spline3d spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(points, spline_degree_);
      const double spline_length = computeLength(spline);

      // Create an extrapolation point at the front of the segment
      if (extrapolation_distance_front_ > std::numeric_limits<double>::epsilon())
      {
        const double u = computeSplineParameterAtDistance(spline, -extrapolation_distance_front_);
        const auto& ref_z = segment.front().matrix().col(2).head<3>();
        Eigen::Isometry3d front = sample(spline, u, ref_z);

        if (normal_offset_distance_front_ > std::numeric_limits<double>::epsilon())
          front *= Eigen::Translation3d(0.0, 0.0, normal_offset_distance_front_);

        segment.insert(segment.begin(), front);
      }

      // Create an extrapolation point at the back of the segment
      if (extrapolation_distance_back_ > std::numeric_limits<double>::epsilon())
      {
        const double u = computeSplineParameterAtDistance(spline, spline_length + extrapolation_distance_back_);
        const auto& ref_z = segment.back().matrix().col(2).head<3>();
        Eigen::Isometry3d back = sample(spline, u, ref_z);

        if (normal_offset_distance_back_ > std::numeric_limits<double>::epsilon())
          back *= Eigen::Translation3d(0.0, 0.0, normal_offset_distance_back_);

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
Node convert<noether::SplineExtrapolationToolPathModifier>::encode(
    const noether::SplineExtrapolationToolPathModifier& val)
{
  Node node;
  node["spline_degree"] = val.spline_degree_;
  node["extrapolation_distance_front"] = val.extrapolation_distance_front_;
  node["normal_offset_distance_front"] = val.normal_offset_distance_front_;
  node["extrapolation_distance_back"] = val.extrapolation_distance_back_;
  node["normal_offset_distance_back"] = val.normal_offset_distance_back_;
  return {};
}

bool convert<noether::SplineExtrapolationToolPathModifier>::decode(const Node& node,
                                                                   noether::SplineExtrapolationToolPathModifier& val)
{
  val.spline_degree_ = YAML::getMember<double>(node, "spline_degree");
  val.extrapolation_distance_front_ = YAML::getMember<double>(node, "extrapolation_distance_front");
  val.normal_offset_distance_front_ = YAML::getMember<double>(node, "normal_offset_distance_front");
  val.extrapolation_distance_back_ = YAML::getMember<double>(node, "extrapolation_distance_back");
  val.normal_offset_distance_back_ = YAML::getMember<double>(node, "normal_offset_distance_back");
  return true;
}
/** @endcond */
}  // namespace YAML
