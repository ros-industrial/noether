#include <noether_gui/widgets/plugin_loader_widget.h>
#include "ui_plugin_loader_widget.h"
#include <noether_gui/widgets/collapsible_area_widget.h>
#include <noether_gui/utils.h>

#include <QMenu>
#include <QMessageBox>
#include <yaml-cpp/yaml.h>

namespace noether
{
template <typename PluginT>
PluginLoaderWidget<PluginT>::PluginLoaderWidget(boost_plugin_loader::PluginLoader loader,
                                                const QString& title,
                                                QWidget* parent)
  : QWidget(parent), ui_(new Ui::PluginLoader()), loader_(std::move(loader))
{
  ui_->setupUi(this);

  ui_->group_box->setTitle(title);
  ui_->combo_box->addItems(getAvailablePlugins<PluginT>(loader_));

  connect(ui_->push_button_add, &QPushButton::clicked, [this](const bool) {
    const QString plugin_name = ui_->combo_box->currentText();
    if (!plugin_name.isEmpty())
    {
      addWidget(plugin_name, {});
    }
  });
}

template <typename PluginT>
void PluginLoaderWidget<PluginT>::addWidget(const QString& plugin_name, const YAML::Node& config)
{
  try
  {
    auto plugin = loader_.createInstance<PluginT>(plugin_name.toStdString());

    auto collapsible_area = new CollapsibleArea(plugin_name, this);
    collapsible_area->setWidget(plugin->create(this, config));
    collapsible_area->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

    ui_->vertical_layout_plugins->addWidget(collapsible_area);

    // Add a right click menu option for deleting this tool path modifier widget
    connect(collapsible_area, &QWidget::customContextMenuRequested, [this, collapsible_area](const QPoint& pos) {
      QMenu context_menu;
      QAction* remove_action = context_menu.addAction("Remove");
      QAction* action = context_menu.exec(collapsible_area->mapToGlobal(pos));
      if (action == remove_action)
      {
        ui_->vertical_layout_plugins->removeWidget(collapsible_area);
        delete collapsible_area;
      }
    });

    // Reset the combo box to the blank value
    ui_->combo_box->setCurrentIndex(0);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Error", QString::fromStdString(ex.what()));
  }
}

template <typename PluginT>
QWidgetList PluginLoaderWidget<PluginT>::getWidgets() const
{
  QWidgetList widgets;
  for (int i = 0; i < ui_->vertical_layout_plugins->count(); ++i)
  {
    QLayoutItem* item = ui_->vertical_layout_plugins->itemAt(i);
    if (item)
    {
      auto collapsible_area = dynamic_cast<CollapsibleArea*>(item->widget());
      if (collapsible_area)
      {
        widgets.push_back(collapsible_area->getWidget());
      }
    }
  }

  return widgets;
}

template <typename PluginT>
void PluginLoaderWidget<PluginT>::configure(const YAML::Node& config)
{
  try
  {
    for (auto it = config.begin(); it != config.end(); ++it)
    {
      addWidget(QString::fromStdString(getEntry<std::string>(*it, "name")), *it);
    }
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Configuration Error", ex.what());
  }
}

template <typename PluginT>
void PluginLoaderWidget<PluginT>::save(YAML::Node& config) const
{
  for (int i = 0; i < ui_->vertical_layout_plugins->count(); ++i)
  {
    QLayoutItem* item = ui_->vertical_layout_plugins->itemAt(i);
    if (item)
    {
      auto collapsible_area = dynamic_cast<const CollapsibleArea*>(item->widget());
      if (collapsible_area)
      {
        auto widget = dynamic_cast<const typename PluginT::WidgetT*>(collapsible_area->getWidget());
        if (widget)
        {
          YAML::Node widget_config;
          widget_config["name"] = collapsible_area->getLabel().toStdString();
          widget->save(widget_config);

          config.push_back(widget_config);
        }
      }
    }
  }
}

}  // namespace noether
