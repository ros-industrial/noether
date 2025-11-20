#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
class FaceSubdivisionMeshModifier : public MeshModifier
{
public:
  FaceSubdivisionMeshModifier(double max_area);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  FaceSubdivisionMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(FaceSubdivisionMeshModifier)

  double max_area_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::FaceSubdivisionMeshModifier)
