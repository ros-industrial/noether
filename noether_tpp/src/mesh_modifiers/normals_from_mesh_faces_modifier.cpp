#include <noether_tpp/mesh_modifiers/normals_from_mesh_faces_modifier.h>
#include <noether_tpp/utils.h>

#include <Eigen/Dense>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace noether
{
std::vector<pcl::PolygonMesh> NormalsFromMeshFacesMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Create a point cloud for the normals and extract its Eigen matrix mapping
  pcl::PointCloud<pcl::Normal> normals_cloud(mesh.cloud.width, mesh.cloud.height);
  auto normals_map = normals_cloud.getMatrixXfMap();

  // Iterate across the polygons, calculating their normals and accumulating the normals onto all adjacent vertices
  for (const auto& p : mesh.polygons)
  {
    // Compute the normal and add to the accumulators for this face's valid vertices
    normals_map({ 0, 1, 2 }, p.vertices).colwise() += getFaceNormal(mesh, p);
  }

  // Normalize the normals
  normals_map({ 0, 1, 2 }, Eigen::all).colwise().normalize();

  // Replace nan values with 0
  normals_map.unaryExpr([](float v) { return std::isfinite(v) ? v : 0.0; });

  // Convert to message type
  pcl::PCLPointCloud2 normals_pc2;
  pcl::toPCLPointCloud2(normals_cloud, normals_pc2);

  // Create output mesh
  pcl::PolygonMesh output = mesh;

  // Concatenate the normals with the nominal mesh information
  if (!pcl::concatenateFields(mesh.cloud, normals_pc2, output.cloud))
    throw std::runtime_error("Failed to concatenate normals into mesh vertex cloud");

  return { output };
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::NormalsFromMeshFacesMeshModifier>::encode(const noether::NormalsFromMeshFacesMeshModifier& val)
{
  return {};
}

bool convert<noether::NormalsFromMeshFacesMeshModifier>::decode(const Node& node,
                                                                noether::NormalsFromMeshFacesMeshModifier& val)
{
  return true;
}
/** @endcond */

}  // namespace YAML
