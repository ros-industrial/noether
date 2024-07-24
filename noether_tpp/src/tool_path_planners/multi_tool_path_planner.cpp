#include <noether_tpp/tool_path_planners/multi_tool_path_planner.h>

namespace noether
{
MultiToolPathPlanner::MultiToolPathPlanner(std::vector<ToolPathPlanner::ConstPtr>&& planners)
  : planners_(std::move(planners))
{
}

ToolPaths MultiToolPathPlanner::plan(const pcl::PolygonMesh& mesh) const
{
  ToolPaths output;
  for (const ToolPathPlanner::ConstPtr& planner : planners_)
  {
    ToolPaths tool_paths = planner->plan(mesh);
    output.insert(output.end(), tool_paths.begin(), tool_paths.end());
  }
  return output;
}

}  // namespace noether
