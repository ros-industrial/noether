#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/fixed_origin_generator_widget.h>
#include "../../../ui_vector3d_editor_widget.h"

#include <noether_tpp/serialization.h>

namespace noether
{
FixedOriginGeneratorWidget::FixedOriginGeneratorWidget(QWidget* parent)
  : BaseWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Origin");
}

void FixedOriginGeneratorWidget::configure(const YAML::Node& config)
{
  auto origin = YAML::getMember<Eigen::Vector3d>(config, "origin");
  ui_->double_spin_box_x->setValue(origin(0));
  ui_->double_spin_box_y->setValue(origin(1));
  ui_->double_spin_box_z->setValue(origin(2));
}

void FixedOriginGeneratorWidget::save(YAML::Node& config) const
{
  config["name"] = "FixedOrigin";
  Eigen::Vector3d origin(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  config["origin"] = origin;
}

}  // namespace noether
