#include <noether_tpp/mesh_modifiers/normal_estimation_pcl.h>
#include <noether_tpp/serialization.h>

#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/common/io.h>

namespace noether
{
NormalEstimationPCLMeshModifier::NormalEstimationPCLMeshModifier(double radius, double vx, double vy, double vz)
  : MeshModifier(), radius_(radius), vx_(vx), vy_(vy), vz_(vz)
{
}

std::vector<pcl::PolygonMesh> NormalEstimationPCLMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Convert to point cloud for normal estimation
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  // Create the normal estimator
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setRadiusSearch(radius_);
  normal_estimator.setViewPoint(vx_, vy_, vz_);

  // Set up the search tree
  auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  normal_estimator.setSearchMethod(tree);

  // Compute normals
  auto normals_cloud = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
  normal_estimator.compute(*normals_cloud);

  // Convert back to a point cloud message
  pcl::PCLPointCloud2 normals;
  pcl::toPCLPointCloud2(*normals_cloud, normals);

  // Copy the input
  pcl::PolygonMesh output = mesh;

  // Concatenate
  if (!pcl::concatenateFields(mesh.cloud, normals, output.cloud))
    throw std::runtime_error("Failed to concatenate normals into mesh vertex cloud");

  return { output };
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::NormalEstimationPCLMeshModifier>::encode(const noether::NormalEstimationPCLMeshModifier& val)
{
  Node node;
  node["radius"] = val.radius_;
  node["vx"] = val.vx_;
  node["vy"] = val.vy_;
  node["vz"] = val.vz_;
  return node;
}

bool convert<noether::NormalEstimationPCLMeshModifier>::decode(const Node& node,
                                                               noether::NormalEstimationPCLMeshModifier& val)
{
  val.radius_ = getMember<double>(node, "radius");
  val.vx_ = getMember<double>(node, "vx");
  val.vy_ = getMember<double>(node, "vy");
  val.vz_ = getMember<double>(node, "vz");
  return true;
}
/** @endcond */

}  // namespace YAML
