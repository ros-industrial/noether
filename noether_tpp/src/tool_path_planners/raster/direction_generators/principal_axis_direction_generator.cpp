#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>
#include <noether_tpp/serialization.h>

#include <boost/make_shared.hpp>
#include <pcl/common/pca.h>

namespace noether
{
PrincipalAxisDirectionGenerator::PrincipalAxisDirectionGenerator(double rotation_offset)
  : rotation_offset_(rotation_offset)
{
}

Eigen::Vector3d PrincipalAxisDirectionGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  auto vertices = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(vertices);
  Eigen::Matrix3d pca_vecs = pca.getEigenVectors().cast<double>();

  // The cutting plane should cut along the largest principal axis (arbitrary decision).
  // Therefore the cutting plane is defined by the direction of the second largest principal axis
  // We then choose to rotate this plane about the smallest principal axis by a configurable angle
  return Eigen::AngleAxisd(rotation_offset_, pca_vecs.col(2).normalized()) * pca_vecs.col(1).normalized();
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::PrincipalAxisDirectionGenerator>::encode(const noether::PrincipalAxisDirectionGenerator& val)
{
  Node node;
  node["rotation_offset"] = val.rotation_offset_;
  return node;
}

bool convert<noether::PrincipalAxisDirectionGenerator>::decode(const Node& node,
                                                               noether::PrincipalAxisDirectionGenerator& val)
{
  val.rotation_offset_ = getMember<double>(node, "rotation_offset");
  return true;
}
/** @endcond */

}  // namespace YAML
