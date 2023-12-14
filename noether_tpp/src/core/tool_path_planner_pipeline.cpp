#include <noether_tpp/core/tool_path_planner_pipeline.h>

#include <utility>  // std::move()

namespace noether
{
ToolPathPlannerPipeline::ToolPathPlannerPipeline(MeshModifier::ConstPtr mesh_mod,
                                                 ToolPathPlanner::ConstPtr planner,
                                                 ToolPathModifier::ConstPtr tool_path_mod)
  : mesh_modifier(std::move(mesh_mod)), planner(std::move(planner)), tool_path_modifier(std::move(tool_path_mod))
{
}

std::vector<ToolPaths> ToolPathPlannerPipeline::plan(pcl::PolygonMesh mesh) const
{
  std::vector<pcl::PolygonMesh> meshes = mesh_modifier->modify(mesh);

  std::vector<ToolPaths> output;
  output.reserve(meshes.size());
  for (const pcl::PolygonMesh& mesh : meshes)
  {
    ToolPaths path = planner->plan(mesh);
    output.push_back(tool_path_modifier->modify(path));
  }

  return output;
}

}  // namespace noether
