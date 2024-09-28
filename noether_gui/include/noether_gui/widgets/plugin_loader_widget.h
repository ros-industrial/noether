#pragma once

#include <boost_plugin_loader/plugin_loader.h>
#include <QWidget>

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
class PluginLoaderWidget : public QWidget
{
public:
  PluginLoaderWidget(boost_plugin_loader::PluginLoader loader, const QString& title, QWidget* parent = nullptr);
  ~PluginLoaderWidget();

  QWidgetList getWidgets() const;

  void configure(const YAML::Node& config);
  void save(YAML::Node& config) const;

  void removeWidgets();

private:
  void addWidget(const QString& plugin_name, const YAML::Node& config);
  void shiftCurrentWidget(const int offset);

  Ui::PluginLoader* ui_;

  /**
   * @brief Container for holding all loaded plugins
   * @details All loaded plugins must be held in scope in order to prevent the plugin libraries from being unloaded
   */
  std::set<std::shared_ptr<PluginT>> plugins_;

  const boost_plugin_loader::PluginLoader loader_;
};

}  // namespace noether
