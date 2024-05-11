#pragma once

#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
class PlaneProjectionMeshModifier : public MeshModifier
{
public:
  PlaneProjectionMeshModifier(double distance_threshold, int max_planes = 1, int min_vertices = 0);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  double distance_threshold_;
  int max_planes_;
  int min_vertices_;
};

}  // namespace noether
