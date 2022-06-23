#pragma once

#include <boost_plugin_loader/plugin_loader.h>
#include <QWidget>

namespace Ui
{
class PluginLoader;
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

  QWidgetList getWidgets() const;

private:
  Ui::PluginLoader* ui_;
  const boost_plugin_loader::PluginLoader loader_;
};

}  // namespace noether
