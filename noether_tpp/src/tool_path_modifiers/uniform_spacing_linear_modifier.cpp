#include <noether_tpp/tool_path_modifiers/uniform_spacing_linear_modifier.h>
#include <noether_tpp/utils.h>

#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformInterpolator.h>

namespace noether
{
UniformSpacingLinearModifier::UniformSpacingLinearModifier(const double point_spacing)
  : ToolPathModifier(), point_spacing_(point_spacing)
{
}

ToolPaths UniformSpacingLinearModifier::modify(ToolPaths tool_paths) const
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

      // Create the transform interpolator
      auto interpolator = vtkSmartPointer<vtkTransformInterpolator>::New();
      interpolator->SetInterpolationTypeToLinear();

      // Add the waypoints to the interpolator
      for (std::size_t i = 0; i < segment.size(); ++i)
      {
        const Eigen::Isometry3d& waypoint = segment.at(i);

        auto mat = vtkSmartPointer<vtkMatrix4x4>::New();
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> map(*mat->Element);
        map = waypoint.matrix();

        interpolator->AddTransform(dists.at(i), mat.Get());
      }

      // Create a helper function for creating interpolated waypoints
      auto create_waypoint = [&interpolator](const double l) -> Eigen::Isometry3d {
        auto vtk_transform = vtkSmartPointer<vtkTransform>::New();
        interpolator->InterpolateTransform(l, vtk_transform.Get());

        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> map(*vtk_transform->GetMatrix()->Element);
        Eigen::Isometry3d waypoint;
        waypoint.matrix() = map;

        return waypoint;
      };

      // Create the new interpolated waypoints
      ToolPathSegment new_segment;
      new_segment.reserve(static_cast<std::size_t>(std::ceil(length / point_spacing_)));

      double l = 0.0;
      while (l < length)
      {
        new_segment.push_back(create_waypoint(l));

        l += point_spacing_;
      }

      // Add the final waypoint
      new_segment.push_back(create_waypoint(length));

      // Add the new segment to the new tool path
      new_segment.shrink_to_fit();
      new_tool_path.push_back(new_segment);
    }

    // Add the new tool path to the output tool paths object
    output.push_back(new_tool_path);
  }

  return output;
}

}  // namespace noether
