#include <noether_gui/plugin_interface.h>
#include <noether_tpp/plugin_interface.h>

#include <boost_plugin_loader/plugin_loader.hpp>

// Macros for converting a non-string target compile definition into a string
#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)

namespace noether
{
std::string MeshModifierWidgetPlugin::getSection() { return STRINGIFY(NOETHER_GUI_MESH_MODIFIER_SECTION); }
std::string ToolPathPlannerWidgetPlugin::getSection() { return STRINGIFY(NOETHER_GUI_TPP_SECTION); }
std::string DirectionGeneratorWidgetPlugin::getSection() { return STRINGIFY(NOETHER_GUI_DIRECTION_GENERATOR_SECTION); }
std::string OriginGeneratorWidgetPlugin::getSection() { return STRINGIFY(NOETHER_GUI_ORIGIN_GENERATOR_SECTION); }
std::string ToolPathModifierWidgetPlugin::getSection() { return STRINGIFY(NOETHER_GUI_TOOL_PATH_MODIFIER_SECTION); }

WidgetFactory::WidgetFactory()
{
  auto loader = std::make_shared<boost_plugin_loader::PluginLoader>();
  loader->search_libraries.emplace_back(NOETHER_PLUGIN_LIB);
  loader->search_libraries.emplace_back(NOETHER_GUI_PLUGIN_LIB);
  loader->search_libraries_env = NOETHER_PLUGIN_LIBS_ENV;
  loader->search_paths_env = NOETHER_PLUGIN_PATHS_ENV;
  loader_ = loader;
}

WidgetFactory::WidgetFactory(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader) : Factory(loader) {}

template <typename PluginT>
BaseWidget* WidgetFactory::createWidget(const std::string& name, const YAML::Node& config, QWidget* parent) const
{
  if (widget_plugins_.find(name) == widget_plugins_.end())
    widget_plugins_[name] = loader_->createInstance<PluginT>(name);

  auto this_shared = std::shared_ptr<const WidgetFactory>(this, [](const WidgetFactory*) {});
  return widget_plugins_[name]->create(config, this_shared, parent);
}

BaseWidget* WidgetFactory::createMeshModifierWidget(const std::string& name,
                                                    const YAML::Node& config,
                                                    QWidget* parent) const
{
  return createWidget<MeshModifierWidgetPlugin>(name, config, parent);
}

BaseWidget* WidgetFactory::createToolPathPlannerWidget(const std::string& name,
                                                       const YAML::Node& config,
                                                       QWidget* parent) const
{
  return createWidget<ToolPathPlannerWidgetPlugin>(name, config, parent);
}

BaseWidget* WidgetFactory::createDirectionGeneratorWidget(const std::string& name,
                                                          const YAML::Node& config,
                                                          QWidget* parent) const
{
  return createWidget<DirectionGeneratorWidgetPlugin>(name, config, parent);
}

BaseWidget* WidgetFactory::createOriginGeneratorWidget(const std::string& name,
                                                       const YAML::Node& config,
                                                       QWidget* parent) const
{
  return createWidget<OriginGeneratorWidgetPlugin>(name, config, parent);
}

BaseWidget* WidgetFactory::createToolPathModifierWidget(const std::string& name,
                                                        const YAML::Node& config,
                                                        QWidget* parent) const
{
  return createWidget<ToolPathModifierWidgetPlugin>(name, config, parent);
}

template std::vector<std::string> Factory::getAvailablePlugins<MeshModifierWidgetPlugin>() const;
template std::vector<std::string> Factory::getAvailablePlugins<ToolPathPlannerWidgetPlugin>() const;
template std::vector<std::string> Factory::getAvailablePlugins<DirectionGeneratorWidgetPlugin>() const;
template std::vector<std::string> Factory::getAvailablePlugins<OriginGeneratorWidgetPlugin>() const;
template std::vector<std::string> Factory::getAvailablePlugins<ToolPathModifierWidgetPlugin>() const;

}  // namespace noether

namespace boost_plugin_loader
{
INSTANTIATE_PLUGIN_LOADER(noether::MeshModifierWidgetPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::ToolPathPlannerWidgetPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::DirectionGeneratorWidgetPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::OriginGeneratorWidgetPlugin)
INSTANTIATE_PLUGIN_LOADER(noether::ToolPathModifierWidgetPlugin)

}  // namespace boost_plugin_loader
