#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/fixed_direction_generator_widget.h>
#include "../../../ui_vector3d_editor_widget.h"

#include <noether_tpp/serialization.h>

namespace noether
{
FixedDirectionGeneratorWidget::FixedDirectionGeneratorWidget(QWidget* parent)
  : DirectionGeneratorWidget(parent), ui_(new Ui::Vector3dEditor())
{
  ui_->setupUi(this);
  ui_->group_box->setTitle("Direction");

  ui_->double_spin_box_x->setValue(1.0);
}

void FixedDirectionGeneratorWidget::configure(const YAML::Node& config)
{
  auto dir = YAML::getMember<Eigen::Vector3d>(config, "direction");
  dir.normalize();

  ui_->double_spin_box_x->setValue(dir.x());
  ui_->double_spin_box_y->setValue(dir.y());
  ui_->double_spin_box_z->setValue(dir.z());
}

void FixedDirectionGeneratorWidget::save(YAML::Node& config) const
{
  config["name"] = "FixedDirection";
  Eigen::Vector3d dir(
      ui_->double_spin_box_x->value(), ui_->double_spin_box_y->value(), ui_->double_spin_box_z->value());
  config["direction"] = dir;
}

}  // namespace noether
