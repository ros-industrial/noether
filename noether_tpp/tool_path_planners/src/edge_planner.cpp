#include <noether_tpp/tool_path_planners/edge/edge_planner.h>

namespace noether_tpp
{
EdgePlanner::EdgePlanner()
  : modifier_(std::move(createDefaultModifier()))
{

}

std::unique_ptr<const OneTimeToolPathModifier> EdgePlanner::createDefaultModifier() { return nullptr; }

ToolPaths EdgePlanner::plan(const pcl::PolygonMesh& mesh) const
{
  return modifier_->modify(planImpl(mesh));
}

} // namespace noether_tpp
