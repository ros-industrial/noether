#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <map>
#include <vector>

namespace noether
{
class UpsamplingMeshModifier : public MeshModifier
{
public:
  UpsamplingMeshModifier(double max_area);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  double max_area_;

  // Helper function to get or create midpoint
  uint32_t getOrCreateMidpoint(uint32_t idx_start,
                               uint32_t idx_end,
                               std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>& vertices,
                               std::map<std::pair<uint32_t, uint32_t>, uint32_t>& edge_midpoints) const;
};

}  // namespace noether
