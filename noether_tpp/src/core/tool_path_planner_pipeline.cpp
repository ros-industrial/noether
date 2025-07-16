#include <noether_tpp/core/tool_path_planner_pipeline.h>
#include <noether_tpp/plugin_interface.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/mesh_modifiers/compound_modifier.h>
#include <noether_tpp/tool_path_modifiers/compound_modifier.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <utility>  // std::move()

namespace noether
{
ToolPathPlannerPipeline::ToolPathPlannerPipeline(MeshModifier::ConstPtr mesh_mod,
                                                 ToolPathPlanner::ConstPtr planner,
                                                 ToolPathModifier::ConstPtr tool_path_mod)
  : mesh_modifier(std::move(mesh_mod)), planner(std::move(planner)), tool_path_modifier(std::move(tool_path_mod))
{
}

ToolPathPlannerPipeline::ToolPathPlannerPipeline(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader,
                                                 const YAML::Node& node)
{
  // Load the mesh modifier
  {
    auto modifiers_config = YAML::getMember<YAML::Node>(node, "mesh_modifiers");

    std::vector<noether::MeshModifier::ConstPtr> modifiers;
    modifiers.reserve(modifiers_config.size());

    for (const YAML::Node& config : modifiers_config)
    {
      auto name = YAML::getMember<std::string>(config, "name");
      auto plugin = loader->createInstance<noether::MeshModifierPlugin>(name);
      modifiers.push_back(plugin->create(config, loader));
    }

    mesh_modifier = std::make_unique<noether::CompoundMeshModifier>(std::move(modifiers));
  }

  // Load the tool path planner
  {
    auto config = YAML::getMember<YAML::Node>(node, "tool_path_planner");
    auto name = YAML::getMember<std::string>(config, "name");
    auto plugin = loader->createInstance<noether::ToolPathPlannerPlugin>(name);
    planner = plugin->create(config, loader);
  }

  // Load the tool path modifiers
  {
    auto modifiers_config = YAML::getMember<YAML::Node>(node, "tool_path_modifiers");

    std::vector<noether::ToolPathModifier::ConstPtr> modifiers;
    modifiers.reserve(modifiers_config.size());

    for (const YAML::Node& config : modifiers_config)
    {
      auto name = YAML::getMember<std::string>(config, "name");
      auto plugin = loader->createInstance<noether::ToolPathModifierPlugin>(name);
      modifiers.push_back(plugin->create(config, loader));
    }

    tool_path_modifier = std::make_unique<noether::CompoundModifier>(std::move(modifiers));
  }
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
