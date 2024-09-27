#pragma once

#include <noether_tpp/tool_path_planners/edge/edge_planner.h>

namespace noether
{
/**
 * @brief Edge tool path planner that creates paths around the boundary edges of a mesh (exterior and interior) as
 * identified by half-edge triangle mesh representation of the mesh
 */
class HalfEdgePlanner : public EdgePlanner
{
public:
  HalfEdgePlanner();

  ToolPaths planImpl(const pcl::PolygonMesh&) const override;
};

struct HalfEdgePlannerFactory : public EdgePlannerFactory
{
  ToolPathPlanner::ConstPtr create() const override;
};

}  // namespace noether
