#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief MeshModifier that assigns vertex normals using mesh faces
 * @details The normal of each vertex is calculated as the average of the normals of all faces
 * which include that vertex. This function handles supra-triangular polygonal faces, but ignores
 * 'faces' that include less than three vertices. Well-behaved on meshes with holes. Note that a
 * mesh with inconsistent face normals will generate inconsistent vertex normals. Note that a
 * non-manifold mesh may produce numerically unstable results.
 */
class NormalsFromMeshFacesMeshModifier : public MeshModifier
{
public:
  using MeshModifier::MeshModifier;

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::NormalsFromMeshFacesMeshModifier)
