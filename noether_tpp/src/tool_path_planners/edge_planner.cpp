#include <noether_tpp/tool_path_planners/edge/edge_planner.h>

#include <utility>  // std::move()

namespace noether
{
ToolPaths EdgePlanner::plan(const pcl::PolygonMesh& mesh) const
{
  ToolPaths tool_paths = planImpl(mesh);

  // To-do: implement the modifications necessary to produce the default behavior

  return tool_paths;
}

}  // namespace noether
