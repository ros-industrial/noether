#include <noether_tpp/mesh_modifiers/normal_estimation_pcl.h>

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
