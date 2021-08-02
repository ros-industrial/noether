#include <noether_tpp/tool_path_planners/edge/edge_planner.h>

#include <utility>  // std::move()

#include <noether_tpp/tool_path_modifiers/default_modifiers.h>

namespace noether
{
ToolPaths EdgePlanner::plan(const pcl::PolygonMesh& mesh) const
{
  DefaultEdgePlannerModifier modifier;
  return modifier.modify(planImpl(mesh));
}

}  // namespace noether
