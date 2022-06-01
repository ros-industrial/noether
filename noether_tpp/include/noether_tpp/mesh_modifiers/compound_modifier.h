#pragma once

#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
class CompoundMeshModifier : public MeshModifier
{
public:
  CompoundMeshModifier(std::vector<MeshModifier::ConstPtr> modifiers);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override final;

protected:
  std::vector<MeshModifier::ConstPtr> modifiers_;
};

}  // namespace noether
