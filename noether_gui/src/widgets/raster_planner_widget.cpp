#include <noether_gui/widgets/raster_planner_widget.h>
#include "ui_raster_planner_widget.h"
#include <noether_gui/plugin_interface.h>
#include <noether_gui/widgets/collapsible_area_widget.h>
#include <noether_gui/utils.h>

#include <QMessageBox>
#include <yaml-cpp/yaml.h>

static const std::string DIRECTION_GENERATOR_KEY = "direction_generator";
static const std::string ORIGIN_GENERATOR_KEY = "origin_generator";
static const std::string LINE_SPACING_KEY = "line_spacing";
static const std::string POINT_SPACING_KEY = "point_spacing";
static const std::string MIN_HOLE_SIZE_KEY = "min_hole_size";

namespace noether
{
RasterPlannerWidget::RasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader, QWidget* parent)
  : ToolPathPlannerWidget(parent), loader_(std::move(loader)), ui_(new Ui::RasterPlanner())
{
  ui_->setupUi(this);

  // Populate the combo boxes
  ui_->combo_box_dir_gen->addItems(getAvailablePlugins<DirectionGeneratorWidgetPlugin>(loader_));
  ui_->combo_box_origin_gen->addItems(getAvailablePlugins<OriginGeneratorWidgetPlugin>(loader_));

  connect(ui_->push_button_dir_gen, &QPushButton::clicked, [this](const bool /*clicked*/) {
    QString text = ui_->combo_box_dir_gen->currentText();
    ui_->group_box_dir_gen->setTitle(text);
    if (text.isEmpty())
      overwriteWidget(ui_->group_box_dir_gen->layout(), ui_->widget_dir_gen, new QWidget(this));
    else
      setDirectionGeneratorWidget(text, {});
  });

  connect(ui_->push_button_origin_gen, &QPushButton::clicked, [this](const bool /*clicked*/) {
    QString text = ui_->combo_box_origin_gen->currentText();
    ui_->group_box_origin_gen->setTitle(text);
    if (text.isEmpty())
      overwriteWidget(ui_->group_box_origin_gen->layout(), ui_->widget_origin_gen, new QWidget(this));
    else
      setOriginGeneratorWidget(text, {});
  });
}

void RasterPlannerWidget::setDirectionGeneratorWidget(const QString& plugin_name, const YAML::Node& config)
{
  try
  {
    auto plugin = loader_.createInstance<DirectionGeneratorWidgetPlugin>(plugin_name.toStdString());

    auto collapsible_area = new CollapsibleArea(plugin_name, this);
    collapsible_area->setWidget(plugin->create(this, config));

    overwriteWidget(ui_->group_box_dir_gen->layout(), ui_->widget_dir_gen, collapsible_area);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Direction Generator Error", QString::fromStdString(ex.what()));
  }
}

void RasterPlannerWidget::setOriginGeneratorWidget(const QString& plugin_name, const YAML::Node& config)
{
  try
  {
    auto plugin = loader_.createInstance<OriginGeneratorWidgetPlugin>(plugin_name.toStdString());

    auto collapsible_area = new CollapsibleArea(plugin_name, this);
    collapsible_area->setWidget(plugin->create(this, config));

    overwriteWidget(ui_->group_box_origin_gen->layout(), ui_->widget_origin_gen, collapsible_area);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Origin Generator Error", QString::fromStdString(ex.what()));
  }
}

void RasterPlannerWidget::configure(const YAML::Node& config)
{
  ui_->double_spin_box_line_spacing->setValue(getEntry<double>(config, LINE_SPACING_KEY));
  ui_->double_spin_box_point_spacing->setValue(getEntry<double>(config, POINT_SPACING_KEY));
  ui_->double_spin_box_minimum_hole_size->setValue(getEntry<double>(config, MIN_HOLE_SIZE_KEY));

  try
  {
    // Direction generator
    auto dir_gen_config = getEntry<YAML::Node>(config, DIRECTION_GENERATOR_KEY);
    QString plugin_name = QString::fromStdString(getEntry<std::string>(dir_gen_config, "name"));
    setDirectionGeneratorWidget(plugin_name, dir_gen_config);
    ui_->group_box_dir_gen->setTitle(plugin_name);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Direction generator configuration error", ex.what());
  }

  try
  {
    // Origin generator
    auto origin_gen_config = getEntry<YAML::Node>(config, ORIGIN_GENERATOR_KEY);
    QString plugin_name = QString::fromStdString(getEntry<std::string>(origin_gen_config, "name"));
    setOriginGeneratorWidget(plugin_name, origin_gen_config);
    ui_->group_box_origin_gen->setTitle(plugin_name);
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Origin generator configuration error", ex.what());
  }
}

void RasterPlannerWidget::save(YAML::Node& config) const
{
  // Direction generator
  try
  {
    YAML::Node dir_gen_config;
    dir_gen_config["name"] = ui_->group_box_dir_gen->title().toStdString();
    getDirectionGeneratorWidget()->save(dir_gen_config);
    config[DIRECTION_GENERATOR_KEY] = dir_gen_config;
  }
  catch (const std::exception&)
  {
  }

  // Origin generator
  try
  {
    YAML::Node origin_gen_config;
    origin_gen_config["name"] = ui_->group_box_origin_gen->title().toStdString();
    getOriginGeneratorWidget()->save(origin_gen_config);
    config[ORIGIN_GENERATOR_KEY] = origin_gen_config;
  }
  catch (const std::exception&)
  {
  }

  config[LINE_SPACING_KEY] = ui_->double_spin_box_line_spacing->value();
  config[POINT_SPACING_KEY] = ui_->double_spin_box_point_spacing->value();
  config[MIN_HOLE_SIZE_KEY] = ui_->double_spin_box_minimum_hole_size->value();
}

DirectionGeneratorWidget* RasterPlannerWidget::getDirectionGeneratorWidget() const
{
  auto collapsible_area = dynamic_cast<CollapsibleArea*>(ui_->widget_dir_gen);
  if (!collapsible_area)
    throw std::runtime_error("No direction generator specified");
  return collapsible_area->getWidget<DirectionGeneratorWidget>();
}

OriginGeneratorWidget* RasterPlannerWidget::getOriginGeneratorWidget() const
{
  auto collapsible_area = dynamic_cast<CollapsibleArea*>(ui_->widget_origin_gen);
  if (!collapsible_area)
    throw std::runtime_error("No origin generator specified");
  return collapsible_area->getWidget<OriginGeneratorWidget>();
}

}  // namespace noether
