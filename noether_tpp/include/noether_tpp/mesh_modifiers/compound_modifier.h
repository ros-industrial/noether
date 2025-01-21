#pragma once

#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief MeshModifier that cascades multiple MeshModifier instances in series
 */
class CompoundMeshModifier : public MeshModifier
{
public:
  CompoundMeshModifier(const CompoundMeshModifier&) = delete;
  CompoundMeshModifier(CompoundMeshModifier&&) = delete;

  CompoundMeshModifier& operator=(CompoundMeshModifier&) = delete;
  CompoundMeshModifier& operator=(CompoundMeshModifier&&) = delete;

  CompoundMeshModifier(std::vector<MeshModifier::ConstPtr> modifiers);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override final;

protected:
  std::vector<MeshModifier::ConstPtr> modifiers_;
};

}  // namespace noether
