#pragma once

#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
class PlaneProjectionMeshModifier : public MeshModifier
{
public:
  PlaneProjectionMeshModifier(double distance_threshold);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  double distance_threshold_;
};

}  // namespace noether
