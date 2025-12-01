#include <noether_tpp/mesh_modifiers/ransac_primitive_fit_modifier.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

#include <numeric>
#include <pcl/sample_consensus/ransac.h>

namespace noether
{
RansacPrimitiveFitMeshModifier::RansacPrimitiveFitMeshModifier(float distance_threshold,
                                                               unsigned min_vertices,
                                                               int max_primitives,
                                                               unsigned max_iterations)
  : MeshModifier()
  , distance_threshold_(distance_threshold)
  , min_vertices_(min_vertices)
  , max_primitives_(max_primitives < 1 ? std::numeric_limits<int>::max() : max_primitives)
  , max_iterations_(max_iterations)
{
}

std::vector<pcl::PolygonMesh> RansacPrimitiveFitMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  findFieldOrThrow(mesh.cloud.fields, "x");
  findFieldOrThrow(mesh.cloud.fields, "y");
  findFieldOrThrow(mesh.cloud.fields, "z");

  // Set up the output data structure
  std::vector<pcl::PolygonMesh> output;

  // Create the SAC primitive model
  auto model = createModel(mesh);

  // Set up the RANSAC model
  auto ransac = pcl::make_shared<pcl::RandomSampleConsensus<pcl::PointXYZ>>(model);
  ransac->setDistanceThreshold(distance_threshold_);
  ransac->setMaxIterations(max_iterations_);

  // Create a vector of indices for the remaining indices to which a model can be fit
  // To start, all indices are remaining (i.e., [0, 1, 2, ..., cloud->size() - 1])
  std::vector<int> all_indices(mesh.cloud.height * mesh.cloud.width);
  std::iota(all_indices.begin(), all_indices.end(), 0);
  std::vector<int> remaining_indices = all_indices;

  /* Detect as many primitives as possible while:
   *   - there are at least enough vertices left to form a new model cluster, and
   *   - we haven't detected more than the maximum number of primitives
   */
  while (remaining_indices.size() >= min_vertices_ && output.size() < max_primitives_)
  {
    // Fit a primitive model to the vertices using RANSAC
    model->setIndices(remaining_indices);
    if (!ransac->computeModel())
      break;

    // Refine the fit model
    if (!ransac->refineModel())
      break;

    // Extract the inliers and ensure there are enough to form a valid model cluster
    std::vector<int> inliers;
    ransac->getInliers(inliers);
    if (inliers.size() < min_vertices_)
      break;

    // Extract and add the sub-mesh
    {
      pcl::PolygonMesh sub_mesh = createSubMesh(mesh, ransac);
      if (!sub_mesh.polygons.empty())
        output.push_back(sub_mesh);
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

namespace YAML
{
/** @cond */
Node convert<noether::RansacPrimitiveFitMeshModifier>::encode(const noether::RansacPrimitiveFitMeshModifier& val)
{
  Node node;
  node["distance_threshold"] = val.distance_threshold_;
  node["min_vertices"] = val.min_vertices_;
  node["max_primitives"] = val.max_primitives_;
  node["max_iterations"] = val.max_iterations_;

  return node;
}

bool convert<noether::RansacPrimitiveFitMeshModifier>::decode(const Node& node,
                                                              noether::RansacPrimitiveFitMeshModifier& val)
{
  val.distance_threshold_ = YAML::getMember<double>(node, "distance_threshold");
  val.min_vertices_ = YAML::getMember<int>(node, "min_vertices");
  val.max_primitives_ = YAML::getMember<int>(node, "max_primitives");
  val.max_iterations_ = YAML::getMember<int>(node, "max_iterations");

  return true;
}
/** @endcond */

}  // namespace YAML
