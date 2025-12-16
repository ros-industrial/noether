#include <noether_tpp/mesh_modifiers/ransac_plane_fit_modifier.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>
#include <noether_tpp/utils.h>
#include <noether_tpp/serialization.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/conversions.h>

namespace noether
{
void projectInPlace(pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& plane_coeffs)
{
  // Find the x, y, and z fields
  auto x_it = findFieldOrThrow(cloud.fields, "x");
  auto y_it = findFieldOrThrow(cloud.fields, "y");
  auto z_it = findFieldOrThrow(cloud.fields, "z");

  auto nx_it = findField(cloud.fields, "normal_x");
  auto ny_it = findField(cloud.fields, "normal_y");
  auto nz_it = findField(cloud.fields, "normal_z");

  // Check that the xyz fields are floats and contiguous
  if ((y_it->offset - x_it->offset != 4) || (z_it->offset - y_it->offset != 4))
    throw std::runtime_error("XYZ fields are not contiguous floats");

  bool update_normals = nx_it != cloud.fields.end() && ny_it != cloud.fields.end() && nz_it != cloud.fields.end();

  Eigen::Vector4f pt_homogeneous;
  pt_homogeneous.setOnes();
  for (std::size_t r = 0; r < cloud.height; ++r)
  {
    for (std::size_t c = 0; c < cloud.width; ++c)
    {
      auto offset = r * cloud.row_step + c * cloud.point_step;
      float* xyz = reinterpret_cast<float*>(cloud.data.data() + offset + x_it->offset);
      Eigen::Map<Eigen::Vector3f> pt(xyz);
      pt_homogeneous.head<3>() = pt;

      // Project the point
      float d = plane_coeffs.dot(pt_homogeneous);
      pt -= d * plane_coeffs.head<3>();

      if (update_normals)
      {
        float* nx = reinterpret_cast<float*>(cloud.data.data() + offset + nx_it->offset);
        *nx = plane_coeffs[0];

        float* ny = reinterpret_cast<float*>(cloud.data.data() + offset + ny_it->offset);
        *ny = plane_coeffs[1];

        float* nz = reinterpret_cast<float*>(cloud.data.data() + offset + nz_it->offset);
        *nz = plane_coeffs[2];
      }
    }
  }
}

std::shared_ptr<pcl::SampleConsensusModel<pcl::PointXYZ>>
RansacPlaneProjectionMeshModifier::createModel(const pcl::PolygonMesh& mesh) const
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  // Set up the RANSAC plane fit model
  return pcl::make_shared<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(cloud);
}

pcl::PolygonMesh RansacPlaneProjectionMeshModifier::createSubMesh(
    const pcl::PolygonMesh& mesh,
    std::shared_ptr<const pcl::RandomSampleConsensus<pcl::PointXYZ>> ransac) const
{
  // Extract the model coefficients
  Eigen::VectorXf plane_coeffs;
  ransac->getModelCoefficients(plane_coeffs);

  std::vector<int> inliers;
  ransac->getInliers(inliers);

  // Extract the inlier sub-mesh
  pcl::PolygonMesh output_mesh = extractSubMeshFromInlierVertices(mesh, inliers);

  // If the mesh is empty, return it instead of trying to project the non-existent inliers
  if (output_mesh.polygons.empty())
    return output_mesh;

  // Compute the unprojected face normals
  Eigen::MatrixX3f normals(output_mesh.polygons.size(), 3);
  for (Eigen::Index i = 0; i < normals.rows(); ++i)
    normals.row(i) = getFaceNormal(output_mesh, output_mesh.polygons[i]);

  // Compute the dot products of each unprojected face normal with the nominal plane normal
  Eigen::VectorXf dot_products = normals * plane_coeffs.head<3>();

  // Invert the fitted plane coefficients if the average face normal and plane normal do not align (i.e., their dot
  // product is < 0)
  if (dot_products.mean() < 0)
  {
    plane_coeffs *= -1;
  }

  // Project the inlier vertices onto the plane
  projectInPlace(output_mesh.cloud, plane_coeffs);

  // Compute the projected face normals
  for (Eigen::Index i = 0; i < normals.rows(); ++i)
    normals.row(i) = getFaceNormal(output_mesh, output_mesh.polygons[i]);

  // Compute the dot products of each projected face normal with the plane normal
  dot_products = normals * plane_coeffs.head<3>();

  // Reverse any faces that do not align with the fitted plane normal
  for (Eigen::Index i = 0; i < dot_products.size(); ++i)
    if (dot_products[i] < 0.0)
      std::reverse(output_mesh.polygons[i].vertices.begin(), output_mesh.polygons[i].vertices.end());

  // Copy the headers from the input mesh to the output mesh
  output_mesh.cloud.header = mesh.cloud.header;
  output_mesh.header = mesh.header;

  return output_mesh;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::RansacPlaneProjectionMeshModifier>::encode(const noether::RansacPlaneProjectionMeshModifier& val)
{
  return convert<noether::RansacPrimitiveFitMeshModifier>::encode(val);
}

bool convert<noether::RansacPlaneProjectionMeshModifier>::decode(const Node& node,
                                                                 noether::RansacPlaneProjectionMeshModifier& val)
{
  return convert<noether::RansacPrimitiveFitMeshModifier>::decode(node, val);
}
/** @endcond */

}  // namespace YAML
