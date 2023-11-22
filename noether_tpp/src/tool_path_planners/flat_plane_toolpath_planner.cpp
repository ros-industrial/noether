#include <math.h>
#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/tool_path_planners/flat_plane_toolpath_planner.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

namespace noether
{
FlatPlaneToolPathPlanner::FlatPlaneToolPathPlanner(const Eigen::Vector2d& plane_dims,
                                                   const Eigen::Vector2d& point_spacing)
  : plane_dims_(plane_dims), point_spacing_(point_spacing)
{
}

ToolPaths FlatPlaneToolPathPlanner::plan(const pcl::PolygonMesh& mesh) const
{
  pcl::PointXYZ mesh_centroid;

  if (mesh.cloud.data.empty())
  {
    pcl::PolygonMesh sparse_mesh;
    pcl::PointCloud<pcl::PointXYZ> sparse_mesh_pcl_cloud;
    pcl::PointXYZ pt;
    sparse_mesh_pcl_cloud.points.push_back(pt);

    pcl::toPCLPointCloud2(sparse_mesh_pcl_cloud, sparse_mesh.cloud);
  }

  else
  {
    // If mesh is not empty, find centroid of the mesh vertices and update the mesh_centroid variable
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    pcl::PointCloud<pcl::PointXYZ> mesh_pcd;
    pcl::fromPCLPointCloud2(mesh.cloud, mesh_pcd);

    for (pcl::PointXYZ point : mesh_pcd.points)
    {
      centroid.add(point);
    }

    centroid.get(mesh_centroid);
  }

  Eigen::Isometry3d mesh_centroid_isom{ Eigen::Isometry3d::Identity() };
  mesh_centroid_isom.translation().x() = mesh_centroid.x;
  mesh_centroid_isom.translation().y() = mesh_centroid.y;
  mesh_centroid_isom.translation().z() = mesh_centroid.z;

  ToolPathSegment segment;
  ToolPath tool_path;
  ToolPaths tool_paths;

  for (size_t i = 0; plane_dims_[0] - point_spacing_[0] * i >= 0; ++i)
  {
    for (size_t j = 0; plane_dims_[1] - point_spacing_[1] * j >= 0; ++j)
    {
      Eigen::Isometry3d pt =
          mesh_centroid_isom * Eigen::Translation3d(i * point_spacing_[0], j * point_spacing_[1], 0.0);
      segment.push_back(pt);
    }
    tool_path.push_back(segment);

    // Clear segment after pushing it to tool path
    segment.clear();
  }
  tool_paths.push_back(tool_path);
  return tool_paths;
}
}  // namespace noether
