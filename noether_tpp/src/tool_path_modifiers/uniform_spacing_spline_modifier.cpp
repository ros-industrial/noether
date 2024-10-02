#include <noether_tpp/tool_path_modifiers/uniform_spacing_spline_modifier.h>
#include <noether_tpp/utils.h>

#include <vtkParametricSpline.h>
#include <vtkPoints.h>
#include <vtkQuaternionInterpolator.h>
#include <vtkSmartPointer.h>

namespace noether
{
/**
 * @brief Sub-class the vtkParametricSpline to have access to the length parameter
 */
class vtkParametricSplineNoether : public vtkParametricSpline
{
public:
  using vtkParametricSpline::vtkParametricSpline;
  vtkTypeMacro(vtkParametricSplineNoether, vtkParametricSpline);

  static vtkParametricSplineNoether* New() { return new vtkParametricSplineNoether(); }

  double getLength() const { return Length; }
};

vtkSmartPointer<vtkPoints> convert(const ToolPathSegment& segment)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (const Eigen::Isometry3d& pose : segment)
    points->InsertNextPoint(pose.translation().data());

  return points;
}

vtkSmartPointer<vtkQuaternionInterpolator> createQuaternionInterpolator(const ToolPathSegment& segment,
                                                                        const std::vector<double>& dists)
{
  auto interpolator = vtkSmartPointer<vtkQuaternionInterpolator>::New();
  interpolator->SetInterpolationTypeToLinear();

  for (std::size_t i = 0; i < segment.size(); ++i)
  {
    Eigen::Quaterniond q(segment.at(i).linear());
    interpolator->AddQuaternion(dists.at(i), q.coeffs().data());
  }

  return interpolator;
}

UniformSpacingSplineModifier::UniformSpacingSplineModifier(const double point_spacing)
  : ToolPathModifier(), point_spacing_(point_spacing)
{
}

ToolPaths UniformSpacingSplineModifier::modify(ToolPaths tool_paths) const
{
  ToolPaths output;
  output.reserve(tool_paths.size());

  for (const ToolPath& tool_path : tool_paths)
  {
    ToolPath new_tool_path;
    new_tool_path.reserve(tool_path.size());

    for (const ToolPathSegment& segment : tool_path)
    {
      // Compute the length of the segment
      double length;
      std::vector<double> dists;
      std::tie(length, dists) = computeLength(segment);

      // Create the position spline
      auto spline = vtkSmartPointer<vtkParametricSplineNoether>::New();
      spline->SetParameterizeByLength(true);
      spline->ClosedOff();
      spline->SetPoints(convert(segment));

      // Create the orientation spline
      vtkSmartPointer<vtkQuaternionInterpolator> q_interpolator = createQuaternionInterpolator(segment, dists);

      // Create a helper function for creating interpolated waypoints
      auto create_waypoint = [&spline, &q_interpolator](const double l) -> Eigen::Isometry3d {
        // Create a new waypoint
        Eigen::Isometry3d waypoint;

        // Evaluate the position spline
        Eigen::Vector3d u =
            Eigen::Vector3d::Ones() * (l / spline->getLength());  // "time" at which to evaluate the spline, on [0, 1]
        double du[9];                                             // Placeholder for the spline derivative
        spline->Evaluate(u.data(), waypoint.translation().data(), du);

        // Evaluate the orientation spline
        Eigen::Quaterniond q;
        q_interpolator->InterpolateQuaternion(l, q.coeffs().data());
        waypoint.linear() = q.toRotationMatrix();

        return waypoint;
      };

      // Create a new tool path segment
      ToolPathSegment new_segment;
      new_segment.reserve(static_cast<std::size_t>(std::ceil(length / point_spacing_)));

      double l = 0.0;
      while (l < length)
      {
        // Add the point to the tool path segment
        new_segment.push_back(create_waypoint(l));

        // Update the point spacing
        l += point_spacing_;
      }

      // Add the last waypoint
      new_segment.push_back(create_waypoint(length));

      // Add the new segment to the new tool path
      new_segment.shrink_to_fit();
      new_tool_path.push_back(new_segment);
    }

    // Add the new tool path to the output tool paths container
    output.push_back(new_tool_path);
  }

  return output;
}

}  // namespace noether
