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

std::vector<ToolPaths> ToolPathPlannerPipeline::plan(pcl::PolygonMesh original_mesh) const
{
  std::vector<pcl::PolygonMesh> meshes;
  try
  {
    meshes = mesh_modifier->modify(original_mesh);
  }
  catch (const std::exception& ex)
  {
    std::throw_with_nested(std::runtime_error("Error invoking mesh modifier in TPP pipeline."));
  }

  std::vector<ToolPaths> output;
  output.reserve(meshes.size());
  for (std::size_t i = 0; i < meshes.size(); ++i)
  {
    const pcl::PolygonMesh& mesh = meshes[i];

    ToolPaths path;
    try
    {
      path = planner->plan(mesh);
    }
    catch (const std::exception& ex)
    {
      std::stringstream ss;
      ss << "Error invoking tool path planner on mesh at index " << i << " in TPP pipeline.";
      std::throw_with_nested(std::runtime_error(ss.str()));
    }

    try
    {
      output.push_back(tool_path_modifier->modify(path));
    }
    catch (const std::exception& ex)
    {
      std::stringstream ss;
      ss << "Error invoking tool path modifier on the tool path at index " << i << " in the TPP pipeline";
      std::throw_with_nested(std::runtime_error(ss.str()));
    }
  }

  return output;
}

}  // namespace noether
