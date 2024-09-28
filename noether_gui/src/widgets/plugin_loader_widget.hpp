#include <noether_gui/widgets/plugin_loader_widget.h>
#include "ui_plugin_loader_widget.h"
#include <noether_gui/utils.h>

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

  QStringList plugins = getAvailablePlugins<PluginT>(loader_);
  plugins.sort();
  ui_->combo_box->addItems(plugins);

  // Add widget
  connect(ui_->tool_button_add, &QAbstractButton::clicked, [this](const bool) {
    const QString plugin_name = ui_->combo_box->currentText();
    if (!plugin_name.isEmpty())
    {
      try
      {
        addWidget(plugin_name, {});
      }
      catch (const std::exception& ex)
      {
        QMessageBox::warning(this, "Error", QString::fromStdString(ex.what()));
      }
    }
  });

  // Remove current widget
  connect(ui_->tool_button_remove, &QAbstractButton::clicked, [this](bool /*clicked*/) {
    int current_row = ui_->list_widget->currentIndex().row();
    QListWidgetItem* item = ui_->list_widget->takeItem(current_row);
    delete item;

    QWidget* w = ui_->stacked_widget->widget(current_row);
    ui_->stacked_widget->removeWidget(w);
    w->deleteLater();
  });

  // Remove all widgets
  connect(ui_->tool_button_remove_all, &QAbstractButton::clicked, this, &PluginLoaderWidget<PluginT>::removeWidgets);

  // Move up and down
  connect(ui_->tool_button_up, &QAbstractButton::clicked, [this](const bool) { shiftCurrentWidget(-1); });
  connect(ui_->tool_button_down, &QAbstractButton::clicked, [this](const bool) { shiftCurrentWidget(1); });

  // Connect changes in the list widget index to changes in the stacked widget index
  connect(ui_->list_widget, &QListWidget::currentRowChanged, ui_->stacked_widget, &QStackedWidget::setCurrentIndex);
}

template <typename PluginT>
PluginLoaderWidget<PluginT>::~PluginLoaderWidget()
{
  /* By default Qt maintains ownership of all widgets and deletes them automatically in the destructor of the QWidget
   * parent class (which occurs after the execution of the destructor of this class). When the plugins are deleted
   * during the destruction of this class (before the QWidget parent class deletes the widgets created by them), the
   * library in which the plugins (and the widgets they create) are defined will be unloaded. Then when the QWidget base
   * class tries to invoke the destructor of the widget(s) created by the plugin(s), a segfault will occur because the
   * library defining those destructor(s) will have already been unloaded.
   *
   * The solution to this issue is to delete all widgets loaded from plugin(s) before deleting the plugins and plugin
   * loader.
   */
  while (ui_->stacked_widget->count() > 0)
  {
    QWidget* widget = ui_->stacked_widget->widget(0);
    ui_->stacked_widget->removeWidget(widget);
    delete widget;
  }
}

template <typename PluginT>
void PluginLoaderWidget<PluginT>::addWidget(const QString& plugin_name, const YAML::Node& config)
{
  auto plugin = loader_.createInstance<PluginT>(plugin_name.toStdString());

  // Store the plugin to prevent it from going out of scope and unloading the plugin library
  plugins_.insert(plugin);

  // Update the list widget and stacked widget
  ui_->list_widget->addItem(plugin_name);
  ui_->stacked_widget->addWidget(plugin->create(this, config));

  // Set the current row of the list widget to trigger an update in the stacked widget
  ui_->list_widget->setCurrentRow(ui_->list_widget->count() - 1);

  // Reset the combo box to the blank value
  ui_->combo_box->setCurrentIndex(0);
}

template <typename PluginT>
void PluginLoaderWidget<PluginT>::shiftCurrentWidget(const int offset)
{
  int current_row = ui_->list_widget->currentIndex().row();
  int new_row = current_row + offset;

  if (new_row > -1 && new_row < ui_->list_widget->count())
  {
    // Update the list widget
    ui_->list_widget->insertItem(new_row, ui_->list_widget->takeItem(current_row));

    // Update the stacked widget
    QWidget* w = ui_->stacked_widget->widget(current_row);
    ui_->stacked_widget->removeWidget(w);
    ui_->stacked_widget->insertWidget(new_row, w);

    // Update the list widget row (which should also trigger an update of the stacked widget)
    ui_->list_widget->setCurrentRow(new_row);
  }
}

template <typename PluginT>
QWidgetList PluginLoaderWidget<PluginT>::getWidgets() const
{
  QWidgetList widgets;
  for (int i = 0; i < ui_->stacked_widget->count(); ++i)
    widgets.push_back(ui_->stacked_widget->widget(i));

  return widgets;
}

template <typename PluginT>
void PluginLoaderWidget<PluginT>::configure(const YAML::Node& config)
{
  removeWidgets();
  for (auto it = config.begin(); it != config.end(); ++it)
  {
    addWidget(QString::fromStdString(getEntry<std::string>(*it, "name")), *it);
  }
}

template <typename PluginT>
void PluginLoaderWidget<PluginT>::save(YAML::Node& config) const
{
  for (int i = 0; i < ui_->stacked_widget->count(); ++i)
  {
    auto widget = dynamic_cast<const typename PluginT::WidgetT*>(ui_->stacked_widget->widget(i));
    if (widget)
    {
      YAML::Node widget_config;
      widget_config["name"] = ui_->list_widget->item(i)->text().toStdString();
      widget->save(widget_config);

      config.push_back(widget_config);
    }
  }
}

template <typename PluginT>
void PluginLoaderWidget<PluginT>::removeWidgets()
{
  // Clear the list widget
  ui_->list_widget->clear();

  // Clear the stacked widget
  for (int i = ui_->stacked_widget->count(); i >= 0; i--)
  {
    QWidget* widget = ui_->stacked_widget->widget(i);
    ui_->stacked_widget->removeWidget(widget);
    widget->deleteLater();
  }
}

}  // namespace noether
