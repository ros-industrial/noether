#include <noether_gui/widgets/origin_generator_widgets.h>
#include "ui_vector3d_editor_widget.h"
#include <noether_gui/utils.h>

#include <noether_tpp/tool_path_planners/raster/origin_generators.h>
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
  config["y"] = ui_->double_spin_box_x->value();
  config["z"] = ui_->double_spin_box_x->value();
}

OriginGenerator::ConstPtr FixedOriginGeneratorWidget::create() const
{
  Eigen::Vector3d origin(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  return std::make_unique<FixedOriginGenerator>(origin);
}

OriginGenerator::ConstPtr CentroidOriginGeneratorWidget::create() const
{
  return std::make_unique<CentroidOriginGenerator>();
}

OriginGenerator::ConstPtr AABBOriginGeneratorWidget::create() const
{
  return std::make_unique<AABBCenterOriginGenerator>();
}

}  // namespace noether
