#include <noether_tpp/tool_path_planners/raster/origin_generators/aabb_origin_generator.h>

#include <pcl/common/common.h>
#include <pcl/conversions.h>

namespace noether
{
Eigen::Vector3d AABBCenterOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(vertices, min, max);
  Eigen::Vector3d range = (max.getArray3fMap() - min.getArray3fMap()).matrix().cast<double>();
  return range / 2.0;
}

}  // namespace noether
