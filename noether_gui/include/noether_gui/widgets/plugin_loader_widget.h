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

class QVBoxLayout;

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

  QWidgetList getWidgets() const;

  void configure(const YAML::Node& config);
  void save(YAML::Node& config) const;

  void removeWidgets();

private:
  void addWidget(const QString& plugin_name, const YAML::Node& config);

  Ui::PluginLoader* ui_;
  QVBoxLayout* widgets_layout_;
  const boost_plugin_loader::PluginLoader loader_;
};

}  // namespace noether
