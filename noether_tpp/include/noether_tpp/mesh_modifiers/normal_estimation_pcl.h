#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief Estimates vertex normals of an input mesh using PCL
 */
class NormalEstimationPCLMeshModifier : public MeshModifier
{
public:
  NormalEstimationPCLMeshModifier(double radius, double vx = 0.0, double vy = 0.0, double vz = 0.0);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  double radius_;
  double vx_;
  double vy_;
  double vz_;

  NormalEstimationPCLMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(NormalEstimationPCLMeshModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::NormalEstimationPCLMeshModifier)
