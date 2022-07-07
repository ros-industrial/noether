#pragma once

#include <boost_plugin_loader/plugin_loader.h>
#include <QLayoutItem>
#include <QLayout>
#include <QStringList>
#include <QWidget>
#include <string>
#include <vector>

namespace noether
{
template <typename PluginT>
inline QStringList getAvailablePlugins(const boost_plugin_loader::PluginLoader& loader)
{
  std::vector<std::string> plugins = loader.getAvailablePlugins<PluginT>();
  QStringList out;
  out.reserve(plugins.size());
  std::transform(plugins.begin(), plugins.end(), std::back_inserter(out), &QString::fromStdString);

  // Insert blank at the beginning
  out.insert(out.begin(), QString{});

  return out;
}

/**
 * @brief Overwrites one widget within a layout with another widget
 */
inline void overwriteWidget(QLayout* layout, QWidget*& from, QWidget* to)
{
  if (!from)
    throw std::runtime_error("From widget is a nullptr");
  if (!to)
    throw std::runtime_error("To widget is a nullptr");
  if (!layout)
    throw std::runtime_error("Layout is a nullptr");

  QLayoutItem* ret = layout->replaceWidget(from, to);
  if (!ret)
    throw std::runtime_error("Widget replacement returned a nullptr");
  if (ret->widget() == from)
    throw std::runtime_error("The widget returned by the replacement did not change");

  // Delete the replaced widget (which is no longer managed), and reset it to the widget that replaced it for future
  // reference
  delete from;
  from = to;
}

}  // namespace noether
