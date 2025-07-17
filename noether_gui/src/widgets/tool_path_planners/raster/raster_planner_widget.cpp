#include <noether_gui/widgets/tool_path_planners/raster/raster_planner_widget.h>
#include "ui_raster_planner_widget.h"
#include <noether_gui/plugin_interface.h>
#include <noether_gui/utils.h>

#include <noether_tpp/serialization.h>
#include <QMessageBox>

static const std::string DIRECTION_GENERATOR_KEY = "direction_generator";
static const std::string ORIGIN_GENERATOR_KEY = "origin_generator";
static const std::string LINE_SPACING_KEY = "line_spacing";
static const std::string POINT_SPACING_KEY = "point_spacing";
static const std::string MIN_HOLE_SIZE_KEY = "min_hole_size";

namespace noether
{
RasterPlannerWidget::RasterPlannerWidget(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader,
                                         QWidget* parent)
  : ToolPathPlannerWidget(parent), loader_(loader), ui_(new Ui::RasterPlanner())
{
  ui_->setupUi(this);

  // Populate the combo boxes
  // Direction generator
  {
    QStringList plugin_names = getAvailablePlugins<DirectionGeneratorWidgetPlugin>(*loader_);
    plugin_names.sort();
    ui_->combo_box_dir_gen->addItems(plugin_names);

    for (const QString& plugin_name : plugin_names)
    {
      if (plugin_name.isEmpty())
      {
        ui_->stacked_widget_dir_gen->addWidget(new QWidget(this));
      }
      else
      {
        auto plugin = loader_->createInstance<DirectionGeneratorWidgetPlugin>(plugin_name.toStdString());
        ui_->stacked_widget_dir_gen->addWidget(plugin->create({}, loader_, this));
      }
    }
  }

  // Origin generator
  {
    QStringList plugin_names = getAvailablePlugins<OriginGeneratorWidgetPlugin>(*loader_);
    plugin_names.sort();
    ui_->combo_box_origin_gen->addItems(plugin_names);

    for (const QString& plugin_name : plugin_names)
    {
      if (plugin_name.isEmpty())
      {
        ui_->stacked_widget_origin_gen->addWidget(new QWidget(this));
      }
      else
      {
        auto plugin = loader_->createInstance<OriginGeneratorWidgetPlugin>(plugin_name.toStdString());
        ui_->stacked_widget_origin_gen->addWidget(plugin->create({}, loader_, this));
      }
    }
  }

  connect(ui_->combo_box_dir_gen,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          ui_->stacked_widget_dir_gen,
          &QStackedWidget::setCurrentIndex);
  connect(ui_->combo_box_origin_gen,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          ui_->stacked_widget_origin_gen,
          &QStackedWidget::setCurrentIndex);
}

void RasterPlannerWidget::configure(const YAML::Node& config)
{
  ui_->double_spin_box_line_spacing->setValue(YAML::getMember<double>(config, LINE_SPACING_KEY));
  ui_->double_spin_box_point_spacing->setValue(YAML::getMember<double>(config, POINT_SPACING_KEY));
  ui_->double_spin_box_minimum_hole_size->setValue(YAML::getMember<double>(config, MIN_HOLE_SIZE_KEY));

  // Direction generator
  {
    const YAML::Node dir_gen_config = config[DIRECTION_GENERATOR_KEY];
    QString plugin_name = QString::fromStdString(YAML::getMember<std::string>(dir_gen_config, "name"));

    const int index = ui_->combo_box_dir_gen->findText(plugin_name);
    if (index >= 0)
    {
      ui_->combo_box_dir_gen->setCurrentIndex(index);
      auto* dir_gen_widget = dynamic_cast<DirectionGeneratorWidget*>(ui_->stacked_widget_dir_gen->widget(index));
      if (dir_gen_widget)
        dir_gen_widget->configure(dir_gen_config);
    }
    else
    {
      ui_->combo_box_dir_gen->setCurrentIndex(0);
      QMessageBox::warning(this, "Error", "Error setting direction generator '" + plugin_name + "'");
    }
  }

  // Origin generator
  {
    const YAML::Node origin_gen_config = config[ORIGIN_GENERATOR_KEY];
    QString plugin_name = QString::fromStdString(YAML::getMember<std::string>(origin_gen_config, "name"));

    const int index = ui_->combo_box_origin_gen->findText(plugin_name);
    if (index >= 0)
    {
      ui_->combo_box_origin_gen->setCurrentIndex(index);
      auto* origin_gen_widget = dynamic_cast<OriginGeneratorWidget*>(ui_->stacked_widget_origin_gen->widget(index));
      if (origin_gen_widget)
        origin_gen_widget->configure(origin_gen_config);
    }
    else
    {
      ui_->combo_box_origin_gen->setCurrentIndex(0);
      QMessageBox::warning(this, "Error", "Error setting origin generator '" + plugin_name + "'");
    }
  }
}

void RasterPlannerWidget::save(YAML::Node& config) const
{
  // Direction generator
  {
    YAML::Node dir_gen_config;
    dir_gen_config["name"] = ui_->combo_box_dir_gen->currentText().toStdString();
    getDirectionGeneratorWidget()->save(dir_gen_config);
    config[DIRECTION_GENERATOR_KEY] = dir_gen_config;
  }

  // Origin generator
  {
    YAML::Node origin_gen_config;
    origin_gen_config["name"] = ui_->combo_box_origin_gen->currentText().toStdString();
    getOriginGeneratorWidget()->save(origin_gen_config);
    config[ORIGIN_GENERATOR_KEY] = origin_gen_config;
  }

  config[LINE_SPACING_KEY] = ui_->double_spin_box_line_spacing->value();
  config[POINT_SPACING_KEY] = ui_->double_spin_box_point_spacing->value();
  config[MIN_HOLE_SIZE_KEY] = ui_->double_spin_box_minimum_hole_size->value();
}

DirectionGeneratorWidget* RasterPlannerWidget::getDirectionGeneratorWidget() const
{
  auto* widget = dynamic_cast<DirectionGeneratorWidget*>(ui_->stacked_widget_dir_gen->currentWidget());
  if (!widget)
    throw std::runtime_error("Invalid direction generator");

  return widget;
}

OriginGeneratorWidget* RasterPlannerWidget::getOriginGeneratorWidget() const
{
  auto* widget = dynamic_cast<OriginGeneratorWidget*>(ui_->stacked_widget_origin_gen->currentWidget());
  if (!widget)
    throw std::runtime_error("Invalid origin generator");

  return widget;
}

}  // namespace noether
