#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
class NormalsFromMeshFacesMeshModifier : public MeshModifier
{
public:
  using MeshModifier::MeshModifier;

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;
};

}  // namespace noether
