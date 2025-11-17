#pragma once

#include <noether_tpp/mesh_modifiers/ransac_primitive_fit_modifier.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief MeshModifier that fits planes to the input mesh and projects the vertices onto the fitted planes
 */
class RansacPlaneProjectionMeshModifier : public RansacPrimitiveFitMeshModifier
{
public:
  using RansacPrimitiveFitMeshModifier::RansacPrimitiveFitMeshModifier;

protected:
  RansacPlaneProjectionMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(RansacPlaneProjectionMeshModifier)

  /**
   * @brief Creates a SAC plane model
   */
  std::shared_ptr<pcl::SampleConsensusModel<pcl::PointXYZ>> createModel(const pcl::PolygonMesh&) const override;

  /**
   * @brief Creates a sub-mesh by projecting the inlier vertices/triangles onto the fitted plane
   */
  pcl::PolygonMesh
  createSubMesh(const pcl::PolygonMesh& mesh,
                std::shared_ptr<const pcl::RandomSampleConsensus<pcl::PointXYZ>> ransac) const override;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::RansacPlaneProjectionMeshModifier)
