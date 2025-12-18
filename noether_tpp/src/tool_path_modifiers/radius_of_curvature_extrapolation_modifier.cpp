#include <noether_tpp/tool_path_modifiers/radius_of_curvature_extrapolation_modifier.h>
#include <noether_tpp/serialization.h>

#include <unsupported/Eigen/Splines>

using Spline = Eigen::Spline<double, 3, 2>;

Eigen::Isometry3d
extrapolate(const Spline& spline, const double t, const double extension_distance, const Eigen::Vector3d& prev_z)
{
  // Evaluate the derivatives of the spline at the given time parameter
  const Eigen::Matrix3d s = spline.derivatives(t, 2);

  // Spline point
  const Eigen::Vector3d p_OP = s.col(0);
  // Spline first derivative; points along the spline
  const Eigen::Vector3d s_dot = s.col(1);
  // Spline second derivative; points towards the center of curvature
  const Eigen::Vector3d s_ddot = s.col(2);

  // Compute the radius of curvature
  const double r = std::pow(s_dot.norm(), 3.0) / (s_dot.cross(s_ddot)).norm();

  // Compute the center of curvature (travel from the spline point in the direction of the spline second derivative by
  // the length of the radius of curvature
  const Eigen::Vector3d v_PC = s_ddot.normalized();
  const Eigen::Vector3d p_OC = p_OP + r * v_PC;

  // To extend the spline, rotate the radius vector about the center of origin:
  // extension_distance = r * theta  --> theta = extension_distance / r
  const double theta = extension_distance / r;
  // At the center of curvature, rotate the radius vector by 180 degrees less the rotation angle,
  const Eigen::Vector3d rotation_axis = s_ddot.cross(s_dot).normalized();
  const Eigen::Vector3d v_CP_ = Eigen::AngleAxisd(M_PI - theta, rotation_axis) * v_PC * r;

  Eigen::Isometry3d pose;

  // Compute the new point by adding the rotated radius vector to the center of rotation
  pose.translation() = p_OC + v_CP_;

  // Compute the rotation
  const Eigen::Vector3d x = v_CP_.cross(rotation_axis);
  const Eigen::Vector3d y = prev_z.cross(x);
  const Eigen::Vector3d z = x.cross(y);

  pose.matrix().col(0).head<3>() = x.normalized();
  pose.matrix().col(1).head<3>() = y.normalized();
  pose.matrix().col(2).head<3>() = z.normalized();

  return pose;
}

namespace noether
{
RadiusOfCurvatureExtrapolationToolPathModifier::RadiusOfCurvatureExtrapolationToolPathModifier(
    const double distance,
    const double normal_offset_distance,
    const bool extend_front,
    const bool extend_back)
  : extrapolation_distance_(distance)
  , normal_offset_distance_(normal_offset_distance)
  , extrapolate_front_(extend_front)
  , extrapolate_back_(extend_back)
{
}

ToolPaths RadiusOfCurvatureExtrapolationToolPathModifier::modify(ToolPaths tool_paths) const
{
  for (ToolPath& tool_path : tool_paths)
  {
    for (ToolPathSegment& segment : tool_path)
    {
      // Extract the spline points
      Eigen::Matrix3Xd points(3, segment.size());
      for (std::size_t i = 0; i < segment.size(); ++i)
        points.col(i) = segment[i].translation();

      // Interpolate the spline
      Eigen::SplineFitting<Spline> spline_fitting;
      Spline spline = spline_fitting.Interpolate(points, 2);

      // Create an extension point at the front of the segment
      if (extrapolate_front_)
      {
        const auto& prev_z = segment.front().matrix().col(2).head<3>();
        Eigen::Isometry3d front = extrapolate(spline, 0.0, -extrapolation_distance_, prev_z);
        front *= Eigen::Translation3d(0.0, 0.0, normal_offset_distance_);
        segment.insert(segment.begin(), front);
      }

      // Create an extension point at the back of the segment
      if (extrapolate_back_)
      {
        const auto& prev_z = segment.back().matrix().col(2).head<3>();
        Eigen::Isometry3d back = extrapolate(spline, 1.0, extrapolation_distance_, prev_z);
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
  val.extrapolation_distance_ = YAML::getMember<double>(node, "extrapolation_distance");
  val.normal_offset_distance_ = YAML::getMember<double>(node, "normal_offset_distance");
  val.extrapolate_front_ = YAML::getMember<bool>(node, "extrapolate_front");
  val.extrapolate_back_ = YAML::getMember<bool>(node, "extrapolate_back");
  return true;
}
/** @endcond */
}  // namespace YAML
