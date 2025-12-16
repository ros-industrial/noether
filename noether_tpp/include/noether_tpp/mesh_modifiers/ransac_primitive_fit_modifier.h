#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace pcl
{
class PointXYZ;

template <typename T>
class SampleConsensusModel;

template <typename T>
class RandomSampleConsensus;
}  // namespace pcl

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief Mesh modifier that fits a specifiable number of model primitives to the input mesh
 * @details This modifier uses RANSAC to sequentially fit primitive models to the input mesh.
 * When a valid model is identified, the inlier vertices are removed from the input mesh into a new sub-mesh.
 * New primitive models are then fitted to the remainder of vertices in the input mesh until either
 * 1) the maximum number of primitive models have been fitted
 * 2) too few inliers remain in the input mesh, or
 * 3) no more valid primitives can be fit to the input mesh.
 */
class RansacPrimitiveFitMeshModifier : public MeshModifier
{
public:
  /**
   * @brief Constructor
   * @param distance_threshold Maximum distance (m) a point can be from the primitive model to be considered an inlier
   * @param min_vertices Minimum number of vertices that a cluster (identfied as a primitive) must have
   * @param max_primitives Maximum number of primitives to detect
   * @param max_iterations Maximum number of RANSAC iterations to perform
   * @param refine_model Flag indicating that a model refinement step should be run after the nominal RANSAC step
   */
  RansacPrimitiveFitMeshModifier(float distance_threshold,
                                 unsigned min_vertices = 1,
                                 int max_primitives = -1,
                                 unsigned max_iterations = 100,
                                 bool refine_model = true);

  virtual ~RansacPrimitiveFitMeshModifier() = default;

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  RansacPrimitiveFitMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(RansacPrimitiveFitMeshModifier)

  /**
   * @brief Creates a sample consensus model for the primitive
   */
  virtual std::shared_ptr<pcl::SampleConsensusModel<pcl::PointXYZ>> createModel(const pcl::PolygonMesh& mesh) const = 0;

  /**
   * @brief Creates a sub-mesh from the results of the model primitive fitting
   * @param mesh Full input mesh
   * @param ransac RANSAC model after successful completion of the model primitive fit
   * @return
   */
  virtual pcl::PolygonMesh
  createSubMesh(const pcl::PolygonMesh& mesh,
                std::shared_ptr<const pcl::RandomSampleConsensus<pcl::PointXYZ>> ransac) const = 0;

  float distance_threshold_;
  unsigned min_vertices_;
  int max_primitives_;
  unsigned max_iterations_;
  bool refine_model_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::RansacPrimitiveFitMeshModifier)
