#include <noether_tpp/mesh_modifiers/plane_projection_modifier.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

namespace noether
{
PlaneProjectionMeshModifier::PlaneProjectionMeshModifier(double distance_threshold)
  : MeshModifier(), distance_threshold_(distance_threshold)
{
}

std::vector<pcl::PolygonMesh> PlaneProjectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  auto model = pcl::make_shared<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(cloud);
  auto ransac = pcl::make_shared<pcl::RandomSampleConsensus<pcl::PointXYZ>>(model);
  ransac->setDistanceThreshold(distance_threshold_);
  ransac->computeModel();

  // Extract the inliers
  std::vector<int> inliers;
  ransac->getInliers(inliers);

  // Get the optimized model coefficients
  Eigen::VectorXf plane_coeffs(4);
  ransac->getModelCoefficients(plane_coeffs);

  // Extract the inlier submesh
  pcl::PolygonMesh output_mesh = extractSubMeshFromInlierVertices(mesh, inliers);

  // Project the inlier vertices onto the plane
  pcl::PointCloud<pcl::PointXYZ> inlier_points;
  pcl::fromPCLPointCloud2(output_mesh.cloud, inlier_points);

  for (pcl::PointXYZ& pt : inlier_points)
  {
    float d = plane_coeffs.dot(pt.getVector4fMap());
    pt.getVector3fMap() -= d * plane_coeffs.head<3>();
  }

  // Replace the vertices with the inlier vertices projected onto the plane
  pcl::toPCLPointCloud2(inlier_points, output_mesh.cloud);

  return { output_mesh };
}

}  // namespace noether
