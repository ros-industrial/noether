#pragma once

#include <noether_gui/widgets/tpp_pipeline_widget.h>

namespace noether
{
class ConfigurableTPPPipelineWidget : public TPPPipelineWidget
{
public:
  static const QString SETTINGS_KEY_DEFAULT_DIRECTORY;
  static const QString SETTINGS_KEY_LAST_FILE;

  ConfigurableTPPPipelineWidget(std::shared_ptr<const WidgetFactory> loader,
                                std::string default_configuration_file_directory = "",
                                QWidget* parent = nullptr);

  void configure(const QString& file);
  void onLoadConfiguration(const bool /*checked*/);
  void onSaveConfiguration(const bool /*checked*/);
};

}  // namespace noether
