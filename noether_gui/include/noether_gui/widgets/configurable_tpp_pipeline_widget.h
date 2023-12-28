#pragma once

#include <QWidget>
#include <boost_plugin_loader/plugin_loader.h>

namespace Ui
{
class ConfigurableTPPPipeline;
}

namespace noether
{
class TPPPipelineWidget;

class ConfigurableTPPPipelineWidget : public QWidget
{
  Q_OBJECT
public:
  ConfigurableTPPPipelineWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent = nullptr);

  TPPPipelineWidget* pipeline_widget;
  void setConfigurationFile(const QString& file);

private:
  void onLoadConfiguration(const bool /*checked*/);
  void onSaveConfiguration(const bool /*checked*/);

  Ui::ConfigurableTPPPipeline* ui_;
};

}  // namespace noether
