#include <noether_tpp/mesh_modifiers/plane_projection_modifier.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>
#include <noether_tpp/utils.h>

#include <numeric>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
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

  for (std::size_t r = 0; r < cloud.height; ++r)
  {
    for (std::size_t c = 0; c < cloud.width; ++c)
    {
      auto offset = r * cloud.row_step + c * cloud.point_step;
      float* xyz = reinterpret_cast<float*>(cloud.data.data() + offset + x_it->offset);
      Eigen::Map<Eigen::Vector4f> pt(xyz);

      // Project the point
      float d = plane_coeffs.dot(pt);
      pt.head<3>() -= d * plane_coeffs.head<3>();

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

PlaneProjectionMeshModifier::PlaneProjectionMeshModifier(double distance_threshold,
                                                         unsigned max_planes,
                                                         unsigned min_vertices)
  : MeshModifier()
  , distance_threshold_(distance_threshold)
  , max_planes_(max_planes < 1 ? std::numeric_limits<int>::max() : max_planes)
  , min_vertices_(min_vertices)
{
}

std::vector<pcl::PolygonMesh> PlaneProjectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Convert the mesh vertices to a point cloud
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  // Set up the output data structure
  std::vector<pcl::PolygonMesh> output;

  // Set up the RANSAC plane fit model
  auto model = pcl::make_shared<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(cloud);
  auto ransac = pcl::make_shared<pcl::RandomSampleConsensus<pcl::PointXYZ>>(model);
  ransac->setDistanceThreshold(distance_threshold_);

  // Create a vector of indices for the remaining indices to which a plane model can be fit
  std::vector<int> remaining_indices(cloud->size());
  std::iota(remaining_indices.begin(), remaining_indices.end(), 0);

  while (remaining_indices.size() >= min_vertices_ && output.size() < max_planes_)
  {
    // Fit a plane model to the vertices using RANSAC
    model->setIndices(remaining_indices);
    if (!ransac->computeModel())
      break;

    // Extract the inliers
    std::vector<int> inliers;
    ransac->getInliers(inliers);
    if (inliers.size() < min_vertices_)
      break;

    // Get the optimized model coefficients
    Eigen::VectorXf plane_coeffs(4);
    ransac->getModelCoefficients(plane_coeffs);

    // Project mesh origin on to plane
    float d = plane_coeffs.dot(Eigen::Vector4f(0.0, 0.0, 0.0, 1.0));
    if (d > 0.0)
      plane_coeffs *= -1;
    // Eigen::Vector3f projected_origin = origin.head<3>() - d * plane_coeffs.head<3>();

    // Extract the inlier submesh
    pcl::PolygonMesh output_mesh = extractSubMeshFromInlierVertices(mesh, inliers);
    if (!output_mesh.polygons.empty())
    {
      // Project the inlier vertices onto the plane
      projectInPlace(output_mesh.cloud, plane_coeffs);

      // Append the extracted and projected mesh to the vector of output meshes
      output.push_back(output_mesh);
    }

    // Remove the inlier indices from the list of remaining indices
    std::size_t num_outliers = remaining_indices.size() - inliers.size();
    if (num_outliers < min_vertices_)
      break;

    std::vector<int> outliers;
    outliers.reserve(num_outliers);
    std::set_difference(remaining_indices.begin(),
                        remaining_indices.end(),
                        inliers.begin(),
                        inliers.end(),
                        std::back_inserter(outliers));
    remaining_indices = outliers;
  }

  return output;
}

}  // namespace noether
