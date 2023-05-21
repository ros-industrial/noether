#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/fixed_origin_generator_widget.h>
#include "ui_vector3d_editor_widget.h"
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/raster/origin_generators/fixed_origin_generator.h>
#include <yaml-cpp/yaml.h>

namespace noether
{
FixedOriginGeneratorWidget::FixedOriginGeneratorWidget(QWidget* parent)
  : OriginGeneratorWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Origin");
}

void FixedOriginGeneratorWidget::configure(const YAML::Node& config)
{
  ui_->double_spin_box_x->setValue(getEntry<double>(config, "x"));
  ui_->double_spin_box_y->setValue(getEntry<double>(config, "y"));
  ui_->double_spin_box_z->setValue(getEntry<double>(config, "z"));
}

void FixedOriginGeneratorWidget::save(YAML::Node& config) const
{
  config["x"] = ui_->double_spin_box_x->value();
  config["y"] = ui_->double_spin_box_y->value();
  config["z"] = ui_->double_spin_box_z->value();
}

OriginGenerator::ConstPtr FixedOriginGeneratorWidget::create() const
{
  Eigen::Vector3d origin(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  return std::make_unique<FixedOriginGenerator>(origin);
}

}  // namespace noether
