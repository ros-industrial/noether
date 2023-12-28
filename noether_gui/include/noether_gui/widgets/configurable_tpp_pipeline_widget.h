#pragma once

#include <QWidget>
#include <boost_plugin_loader/plugin_loader.h>
#include <noether_tpp/core/tool_path_planner_pipeline.h>

namespace Ui
{
class ConfigurableTPPPipeline;
}

namespace YAML
{
class Node;
}

namespace noether
{
class TPPPipelineWidget;

class ConfigurableTPPPipelineWidget : public QWidget
{
  Q_OBJECT
public:
  ConfigurableTPPPipelineWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent = nullptr);

  void setConfigurationFile(const QString& file);

  ToolPathPlannerPipeline createPipeline() const;
  void configure(const YAML::Node& config);
  void save(YAML::Node& config) const;

private:
  void onLoadConfiguration(const bool /*checked*/);
  void onSaveConfiguration(const bool /*checked*/);

  Ui::ConfigurableTPPPipeline* ui_;
  TPPPipelineWidget* pipeline_widget_;
};

}  // namespace noether
