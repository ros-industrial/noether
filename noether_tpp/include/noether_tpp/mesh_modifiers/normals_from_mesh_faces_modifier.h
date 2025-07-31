#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief MeshModifier that assigns vertex normals using mesh faces
 * @details
 */
class NormalsFromMeshFacesMeshModifier : public MeshModifier
{
public:
  using MeshModifier::MeshModifier;

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::NormalsFromMeshFacesMeshModifier)
