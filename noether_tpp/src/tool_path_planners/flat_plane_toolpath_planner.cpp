#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/tool_path_planners/flat_plane_toolpath_planner.h>
#include <math.h>

namespace noether
{
FlatPlaneToolPathPlanner::FlatPlaneToolPathPlanner(const Eigen::Vector2d& plane_dims,
                                                   const Eigen::Vector2d& point_spacing,
                                                   Eigen::Isometry3d offset)
  : plane_dims_(plane_dims), point_spacing_(point_spacing), offset_(offset)
{
}
ToolPaths FlatPlaneToolPathPlanner::plan(const pcl::PolygonMesh& /*mesh*/) const
{
  ToolPathSegment segment;
  ToolPath tool_path;
  ToolPaths tool_paths;

  Eigen::Isometry3d eigen_pose = offset_;
  for (size_t i = 0; plane_dims_[0] - point_spacing_[0] * i >= 0; i++)
  {
    for (size_t j = 0; plane_dims_[1] - point_spacing_[1] * j >= 0; j++)
    {
      Eigen::Isometry3d pt = eigen_pose * Eigen::Translation3d(i * point_spacing_[0], j * point_spacing_[1], 0.0);
      segment.push_back(pt);
    }
    if (i % 2 != 0)
    {
      std::reverse(segment.begin(), segment.end());
      tool_path.push_back(segment);
    }
    else
    {
      tool_path.push_back(segment);
    }
    segment.clear();
  }
  tool_paths.push_back(tool_path);
  return tool_paths;
}
}  // namespace noether
