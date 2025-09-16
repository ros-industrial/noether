#pragma once

#include <noether_gui/plugin_interface.h>

namespace Ui
{
class PluginLoader;
}

namespace YAML
{
class Node;
}

namespace noether
{
/**
 * @brief Widget for loading widget plugins
 */
template <typename PluginT>
class PluginLoaderWidget : public BaseWidget
{
public:
  PluginLoaderWidget(std::shared_ptr<const WidgetFactory> factory, const QString& title, QWidget* parent = nullptr);
  ~PluginLoaderWidget();

  QWidgetList getWidgets() const;

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

  void removeWidgets();

protected:
  void addWidget(const QString& plugin_name, const YAML::Node& config);
  void shiftCurrentWidget(const int offset);

  Ui::PluginLoader* ui_;
  std::shared_ptr<const WidgetFactory> factory_;
};

}  // namespace noether
