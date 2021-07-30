#include <noether_tpp/core/tool_path_planner_pipeline.h>

#include <utility>  // std::move()

namespace noether
{
ToolPathPlannerPipeline::ToolPathPlannerPipeline(std::unique_ptr<MeshModifier>&& mesh_mod,
                                                 std::unique_ptr<ToolPathPlanner>&& planner,
                                                 std::unique_ptr<ToolPathModifier>&& tool_path_mod)
  : mesh_modifier_(std::move(mesh_mod)), planner_(std::move(planner)), tool_path_modifier_(std::move(tool_path_mod))
{
}

std::vector<ToolPaths> ToolPathPlannerPipeline::plan(pcl::PolygonMesh mesh) const
{
  std::vector<pcl::PolygonMesh> meshes = mesh_modifier_->modify(mesh);

  std::vector<ToolPaths> output;
  output.reserve(meshes.size());
  for (const pcl::PolygonMesh& mesh : meshes)
  {
    ToolPaths path = planner_->plan(mesh);
    output.push_back(tool_path_modifier_->modify(path));
  }

  return output;
}

}  // namespace noether
