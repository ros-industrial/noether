#pragma once

#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
class PlaneProjectionMeshModifier : public MeshModifier
{
public:
  PlaneProjectionMeshModifier(double distance_threshold, unsigned max_planes = 1, unsigned min_vertices = 1);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  double distance_threshold_;
  unsigned max_planes_;
  unsigned min_vertices_;
};

}  // namespace noether
