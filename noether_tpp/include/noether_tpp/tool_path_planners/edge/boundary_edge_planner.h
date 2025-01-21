#pragma once

#include <noether_tpp/tool_path_planners/edge/edge_planner.h>

namespace noether
{
/**
 * @ingroup edge_planners
 * @brief Edge tool path planner that creates paths around the boundary edges of a mesh (exterior and interior) as
 * identified by half-edge triangle mesh representation of the mesh
 */
class BoundaryEdgePlanner : public EdgePlanner
{
public:
  BoundaryEdgePlanner();

  ToolPaths planImpl(const pcl::PolygonMesh&) const override;
};

struct BoundaryEdgePlannerFactory : public EdgePlannerFactory
{
  ToolPathPlanner::ConstPtr create() const override;
};

}  // namespace noether
