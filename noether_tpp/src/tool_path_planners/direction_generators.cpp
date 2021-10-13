#include <noether_tpp/tool_path_planners/raster/direction_generators.h>

#include <boost/make_shared.hpp>
#include <pcl/common/pca.h>

namespace noether
{
FixedDirectionGenerator::FixedDirectionGenerator(const Eigen::Vector3d& direction) : direction_(direction.normalized())
{
}

Eigen::Vector3d FixedDirectionGenerator::generate(const pcl::PolygonMesh&) const { return direction_; }

std::unique_ptr<DirectionGenerator> FixedDirectionGenerator::clone() const
{
  return std::make_unique<FixedDirectionGenerator>(direction_);
}

PrincipalAxisDirectionGenerator::PrincipalAxisDirectionGenerator(double rotation_offset)
  : rotation_offset_(rotation_offset)
{
}

Eigen::Vector3d PrincipalAxisDirectionGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  auto vertices = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(vertices);
  Eigen::Matrix3d pca_vecs = pca.getEigenVectors().cast<double>();

  // The cutting plane should cut along the largest principal axis (arbitrary decision).
  // Therefore the cutting plane is defined by the direction of the second largest principal axis
  // We then choose to rotate this plane about the smallest principal axis by a configurable angle
  return Eigen::AngleAxisd(rotation_offset_, pca_vecs.col(2).normalized()) * pca_vecs.col(1).normalized();
}

std::unique_ptr<DirectionGenerator> PrincipalAxisDirectionGenerator::clone() const
{
  return std::make_unique<PrincipalAxisDirectionGenerator>(rotation_offset_);
}

}  // namespace noether
