#pragma once

#include <noether_gui/widgets/tpp_pipeline_widget.h>

namespace noether
{
class ConfigurableTPPPipelineWidget : public TPPPipelineWidget
{
public:
  ConfigurableTPPPipelineWidget(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader,
                                std::string default_configuration_file_directory = "",
                                QWidget* parent = nullptr);

  void setConfigurationFile(const QString& file);
  void configure(const QString& file);
  void onLoadConfiguration(const bool /*checked*/);
  void onSaveConfiguration(const bool /*checked*/);

protected:
  std::string default_configuration_file_directory_;
};

}  // namespace noether
