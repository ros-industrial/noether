#include <noether_tpp/tool_path_planners/raster/origin_generators.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

namespace noether
{
FixedOriginGenerator::FixedOriginGenerator(const Eigen::Vector3d& origin) : origin_(origin) {}

Eigen::Vector3d FixedOriginGenerator::generate(const pcl::PolygonMesh& mesh) const { return origin_; }

Eigen::Vector3d CentroidOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);
  pcl::PointXYZ centroid;
  if (pcl::computeCentroid(vertices, centroid) < 1)
    throw std::runtime_error("Failed to compute mesh centroid");

  return centroid.getArray3fMap().matrix().cast<double>();
}

Eigen::Vector3d AABBCenterOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(vertices, min, max);
  Eigen::Vector3d range = (max.getArray3fMap() - min.getArray3fMap()).matrix().cast<double>();
  return range / 2.0;
}

OffsetOriginGenerator::OffsetOriginGenerator(std::unique_ptr<const OriginGenerator>&& generator,
                                             const Eigen::Vector3d& offset)
  : generator_(std::move(generator)), offset_(offset)
{
}

Eigen::Vector3d OffsetOriginGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  Eigen::Vector3d origin = generator_->generate(mesh);
  return origin + offset_;
}

}  // namespace noether
